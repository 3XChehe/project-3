hiện drone của tôi đang sử dụng mã nguồn tự động set acc offset trong folder main/ESP32_FC/CODE_FC/DroneFC_CRSF_DShot_ManualAcc
nó hiện đang bị trôi và có vẻ như ngẫu nhiên,sau khi bay sang trái, phải, trước, sau thì drone sẽ trôi thiên hướng về phía vừa bay sang và có lẽ nó cũng đang bị nhiễu IMU nhiễu do mạch băm xung từ ESC đặt ở tầng dưới, tức là có 3 tầng là esc, đặt song song ở trên là ICM 20602 , BMP388 và tầng trên cùng là esp32 với RC
hãy suy nghĩ và giúp tôi xử lý vấn đề này

Điều chỉnh hiệu chuẩn con quay và theo dõi độ lệch khi đứng yên.
Phân tích drone trôi sau khi bay
Mã ManualAcc dùng offset cố định (+0.0152, +0.0141, +0.0337), không tự đo lại lúc cắm pin. Đọc kỹ code, tôi thấy triệu chứng “trôi về phía vừa bay” khớp nhất với hai nguyên nhân dưới đây. Rung và nhiễu có góp phần nhưng không phải thủ phạm chính.

1. Nguyên nhân, xếp theo khả năng
A. Bộ lọc góc tin gia tốc kế ngay cả khi drone đang gia tốc (khả năng cao nhất)

Kalman 1D trong imu_update() luôn dùng góc từ accel làm phép đo, với R = 3². Độ lợi khoảng 0.0007 mỗi vòng, tức hằng số thời gian khoảng 3 giây.
Khi bạn nghiêng để bay sang trái, lực đẩy cánh quạt nằm dọc trục thân máy. Accel đo lực đẩy + trọng lực nên gần như luôn báo “đang phẳng”, dù thực tế drone đang nghiêng.
Trong khoảng 2 giây bay, bộ lọc bị accel kéo góc ước lượng về 0. Controller thì bám theo góc ước lượng, nên thân máy thật nghiêng nhiều hơn góc ước lượng.
Khi bạn thả cần, controller đưa góc ước lượng về 0, nhưng thân máy thật vẫn còn nghiêng theo hướng vừa bay. Drone trôi tiếp vài giây cho tới khi accel kéo ước lượng về đúng.
Rung động làm accel nhiễu thêm và sai lệch này càng lớn.
B. Cần gạt không về đúng tâm và code không có vùng chết

Trong rc_fetch_and_map(), 1 µs lệch tâm tương đương 0.04° góc đặt. Cần gạt Hall có ma sát và hồi trễ, nên sau khi gạt sang phải nó thường dừng lệch tâm một chút về phía đã gạt.
Chỉ cần lệch 10–15 µs là drone bị nghiêng khoảng 0.5° liên tục về đúng hướng vừa bay. Triệu chứng này khớp hoàn toàn với “thiên hướng về phía vừa bay” và cảm giác ngẫu nhiên.
C. Cân bằng gốc (trim) và bias gyro chưa chuẩn

Offset accel đo trên mặt bàn, nhưng điều quan trọng là mặt phẳng của khung khi bay. Bo IMU gắn trên đệm cao su thường lệch 0.5–1°. Offset hiện tại (0.015 g) đã tương đương khoảng 0.9°.
imu_calibrate_gyro() lấy 500 mẫu cách nhau 10 µs, nên chỉ đo trong khoảng 25 ms. Gyro đang lọc 5 Hz nên khoảng đo ngắn như vậy cho bias kém chính xác. Gyro còn nóng dần sau khi cấp nguồn nên bias trôi theo. Khi tin accel ít đi, sai số bias sẽ lộ ra.
IMU_DLPF_GYRO = 0x06 là gyro lọc 5 Hz, rất thấp, gây trễ pha lớn cho vòng rate. Drone phản ứng chậm với nhiễu và gió.
D. Rung và nhiễu

IMU là MEMS, ít nhạy với EMI. Cái ảnh hưởng nhiều hơn là rung cơ học (cánh không cân bằng, motor, dây kéo bo) và nhiễu trên đường SPI/nguồn.
Nhiễu SPI gây ra các giá trị gai đột ngột. Mạch ESC ở tầng dưới nằm sát dây SPI/DShot, có thể gây nhiễu chéo.
E. Trôi quán tính là bình thường

Angle mode không có cảm biến vận tốc (GPS hay optical flow), nên thả cần thì drone vẫn lướt theo đà. Không thể loại bỏ hẳn phần này, chỉ phân biệt được với lỗi ước lượng bằng các phép thử bên dưới.

2. Hai phép thử để xác định nguyên nhân
Tháo cánh quạt khi thử.

Thử rung và nhiễu: bật DEBUG_ATTITUDE, đặt drone phẳng trên bàn, ghi lại roll và pitch. Arm, đẩy ga khoảng 50% trong 10 giây rồi so sánh.
Nếu góc lệch hơn khoảng 1° so với lúc motor tắt, nguồn gốc là rung hoặc nhiễu, ưu tiên làm mục 4.
Nếu góc không đổi, phần cứng khá ổn, tập trung mục 3.
Thử cần gạt: bật DEBUG_RC. Gạt roll sang phải hết cỡ, thả ra, ghi ch1. Làm tương tự cho trái, trước, sau. Nếu giá trị thả ra không về 1500±3 thì nguyên nhân B đã xác nhận.
3. Sửa trong code
3.1. Vùng chết và trim cho cần gạt (thay thế rc_fetch_and_map() trong control_task.ino)

cpp
#define RC_DEADBAND_US   8      // vùng chết quanh tâm, đơn vị us
#define ROLL_TRIM_DEG    0.0f   // chỉnh nếu hover vẫn trôi đều một hướng
#define PITCH_TRIM_DEG   0.0f

static float stick_norm(int us) {      // trả -500..+500 sau khi bỏ vùng chết
  int d = us - 1500;
  if (d >  RC_DEADBAND_US) return (float)(d - RC_DEADBAND_US) * 500.0f / (500 - RC_DEADBAND_US);
  if (d < -RC_DEADBAND_US) return (float)(d + RC_DEADBAND_US) * 500.0f / (500 - RC_DEADBAND_US);
  return 0.0f;
}

void rc_fetch_and_map() {
  if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
    rc_ok = shared_rc_ok;
    for (int i = 1; i <= 16; i++) rc_ch[i] = shared_rc_ch[i];
    xSemaphoreGive(mtx_rc);
  }
  for (int i = 5; i <= 8; i++) rc_ch[i] = rc_switch_position(rc_ch[i]);

  const float a = 0.05f;
  roll_sp_deg     = roll_sp_deg     * (1 - a) + ( stick_norm(rc_ch[CH_ROLL])     / (500.0f / LIM_TILT_DEG)     + ROLL_TRIM_DEG)  * a;
  pitch_sp_deg    = pitch_sp_deg    * (1 - a) + ( stick_norm(rc_ch[CH_PITCH])    / (500.0f / LIM_TILT_DEG)     + PITCH_TRIM_DEG) * a;
  yaw_rate_sp_dps = yaw_rate_sp_dps * (1 - a) + (-stick_norm(rc_ch[CH_YAW])      / (500.0f / LIM_YAW_RATE_DPS))                  * a;
  climb_sp_cms    = climb_sp_cms    * (1 - a) + ( stick_norm(rc_ch[CH_THROTTLE]) / (500.0f / LIM_CLIMB_RATE_CMS))                * a;
}
Nên calibrate lại cần gạt trong EdgeTX (SYS → Hardware → Sticks calibration).

3.2. Giảm độ tin accel khi đang bay và đang gia tốc (imu_icm20602.ino)

Thay imu_update() và imu_kalman_1d():

cpp
#define IMU_ACC_VAR_DEG2   9.0f      // 3^2, ứng với tin tưởng accel 100%
static float imu_acc_trust = 1.0f;   // 0..1, control_task đặt mỗi vòng
void imu_set_acc_trust(float t) { imu_acc_trust = constrain(t, 0.02f, 1.0f); }

void imu_update() {
  imu_read_raw();

  float ax = imu_acc_x_g + ACC_OFFSET_X_G;
  float ay = imu_acc_y_g + ACC_OFFSET_Y_G;
  float az = imu_acc_z_g + ACC_OFFSET_Z_G;
  float dev = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);   // lệch khỏi 1 g

  // |a| lệch 1 g (đang gia tốc, rung mạnh) -> tin accel ít đi
  float k = dev / 0.04f;
  float trust = imu_acc_trust / (1.0f + k * k);

  imu_kalman_1d(att_roll_deg,  att_roll_var,
                imu_rate_roll_dps  - gyro_bias_roll_dps,  acc_roll_deg,  trust);
  imu_kalman_1d(att_pitch_deg, att_pitch_var,
                imu_rate_pitch_dps - gyro_bias_pitch_dps, acc_pitch_deg, trust);
}

void imu_kalman_1d(float &state_deg, float &variance,
                   float rate_dps, float measure_deg, float trust) {
  state_deg += DT_CTRL * rate_dps;
  variance  += DT_CTRL * DT_CTRL * 1.0f * 1.0f;

  float r    = IMU_ACC_VAR_DEG2 / (trust * trust);
  float gain = variance / (variance + r);
  state_deg += gain * (measure_deg - state_deg);
  variance   = (1.0f - gain) * variance;
}
Trong control_task.ino, thêm biến toàn cục uint32_t acc_hold_until_ms = 0; và đặt đoạn sau ngay sau rc_fetch_and_map();:

cpp
    // Đang ra lệnh nghiêng (và 1.5 s sau đó) thì accel không đáng tin
    if (fabsf(roll_sp_deg) > 2.0f || fabsf(pitch_sp_deg) > 2.0f)
      acc_hold_until_ms = millis() + 1500;

    float trust = !armed ? 1.0f : (millis() < acc_hold_until_ms ? 0.1f : 0.5f);
    imu_set_acc_trust(trust);
Ý nghĩa: trên mặt đất tin accel hoàn toàn, khi đang bay thường chỉ tin một nửa, khi đang vận động tin 10%. Khi đó hằng số thời gian kéo về của accel dài ra vài chục giây, nên gyro dẫn dắt trong lúc vận động. Cái giá phải trả là bias gyro phải tốt, xem 3.3.

3.3. Hiệu chỉnh gyro tốt hơn (imu_icm20602.ino)

Trong imu_calibrate_gyro(), đổi delayMicroseconds(10); thành delay(2);. Mỗi đợt sẽ kéo dài khoảng 1 giây, đủ để bias không bị ảnh hưởng bởi lọc 5 Hz.

Thêm hàm bám bias khi chưa arm, gọi mỗi vòng điều khiển lúc !armed:

cpp
void imu_track_gyro_bias() {
  const float a = 0.0005f;   // ~4 s ở 500 Hz
  float dr = imu_rate_roll_dps  - gyro_bias_roll_dps;
  float dp = imu_rate_pitch_dps - gyro_bias_pitch_dps;
  float dy = imu_rate_yaw_dps   - gyro_bias_yaw_dps;
  if (fabsf(dr) < 1.5f && fabsf(dp) < 1.5f && fabsf(dy) < 1.5f) {   // đang đứng yên
    gyro_bias_roll_dps  += a * dr;
    gyro_bias_pitch_dps += a * dp;
    gyro_bias_yaw_dps   += a * dy;
  }
}
Trong control_task.ino gọi if (!armed) imu_track_gyro_bias(); sau khối arm/disarm. Cách này bù được phần bias trôi theo nhiệt trong lúc chờ cất cánh. Tốt nhất là cắm pin rồi để drone nằm yên khoảng 30–60 giây cho IMU ấm trước khi bay.

3.4. Đọc IMU một lần 14 byte (giảm rủi ro nhiễu SPI và lấy mẫu accel/gyro cùng thời điểm)

cpp
static bool imu_read_burst(int16_t &ax, int16_t &ay, int16_t &az,
                           int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[14];
  SPI.beginTransaction(imu_spi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(REG_ACCEL_XOUT_H | 0x80);
  for (int i = 0; i < 14; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();

  bool all_ff = true, all_00 = true;           // bus treo / không có phản hồi
  for (int i = 0; i < 14; i++) { if (b[i] != 0xFF) all_ff = false; if (b[i] != 0x00) all_00 = false; }
  if (all_ff || all_00) return false;

  ax = (int16_t)((b[0] << 8) | b[1]);   ay = (int16_t)((b[2] << 8) | b[3]);   az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);   gy = (int16_t)((b[10] << 8) | b[11]); gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}
Trong imu_read_raw(), thay hai lần đọc imu_read_accel_raw và imu_read_gyro_raw bằng một lần gọi hàm này. Nếu trả false thì giữ nguyên số liệu vòng trước.

3.5. Bộ lọc gyro (làm sau cùng)

Gyro 5 Hz làm vòng rate rất chậm. Khi đã giảm rung xong, thử IMU_DLPF_GYRO 0x03 (khoảng 41 Hz) và có thể giảm KD_RATE_* một chút. Bước này đổi hành vi vòng PID nên phải thử có buộc dây. Các giá trị DLPF tôi nêu theo trí nhớ từ datasheet, bạn nên đối chiếu lại bảng CONFIG của ICM-20602 trước khi đổi.

3.6. Trước khi bay

Tắt DEBUG_ACC_OFFSET và mọi DEBUG_*. Hiện tại file .ino đang bật DEBUG_ACC_OFFSET.

4. Phần cứng
IMU: dùng đệm cao su hoặc xốp mềm riêng cho bo IMU. Dây nguồn và dây ESC không được kéo hoặc đè lên bo, vì dây làm rung truyền ngược vào IMU.
Cánh và motor: cân bằng cánh, kiểm tra ốc motor và vòng bi. Thử nghiệm ở mục 2 sẽ cho biết rung có ảnh hưởng hay không.
Dây SPI: ngắn, xoắn cùng một dây GND. Giữ cách xa dây DShot (GPIO 25/26/27/33) và dây motor. Nếu vẫn có gai, hạ IMU_SPI_HZ xuống 4 MHz, vòng 500 Hz vẫn dư thời gian.
Nguồn IMU: thêm tụ 100 nF gốm cộng 10 µF sát chân VCC của IMU. IMU đang ăn 3V3 từ LDO trên ESP32, nên cũng nên có tụ lọc 5V gần BEC.
5. Thứ tự đề xuất
Làm hai phép thử ở mục 2 để biết đang gặp vấn đề nào.
Áp dụng 3.1 (vùng chết) và 3.2 (giảm tin accel), bay thử có buộc dây.
Áp dụng 3.3 (gyro bias) và 3.4 (đọc burst).
Nếu hover vẫn trôi đều một hướng, chỉnh ROLL_TRIM_DEG/PITCH_TRIM_DEG từng bước 0.3°.
Cuối cùng mới thử nới IMU_DLPF_GYRO, và xử lý phần cứng nếu ở bước 1 thấy rung lớn.
Phần trôi còn lại sau các bước trên là quán tính của Angle mode, chỉ khắc phục được khi có cảm biến vận tốc hoặc vị trí. Nếu chạy xong các bước trên mà vẫn trôi, hãy gửi lại log DEBUG_ATTITUDE và DEBUG_RC đo ở bước 1, tôi sẽ đọc tiếp.

Lưu ý thêm: file GIAI_THICH_PID_VA_CO_CHE_DIEU_KHIEN.md mô tả mixer cũ (motor phía cao tăng lực), khác với mixer hiện tại trong control_task.ino. Mixer hiện tại đã đúng dấu (nghiêng phải thì motor phải tăng), nên đừng dựa vào file đó khi debug.

hãy xem và đánh giá rủi ro bị windup của các chế độ bay

Tuning hover and throttle bleed thresholds for stability.
Đánh giá rủi ro windup theo chế độ bay
Tôi đọc ManualAcc/pid.ino và control_task.ino. Mọi con số về ngưỡng tay ga và độ dự trữ công suất dưới đây là ước lượng từ code. Tôi chưa biết ga hover thật của bạn, nên cần đo lại (xem cuối bài).

1. Hiện trạng bộ chống windup
Tầng góc (pid_tilt_*): KI = 0 nên không có windup.
pid_step (rate roll/pitch/yaw): có chặn tích phân theo I_LIM và dừng tích phân khi đầu ra PID chạm U_LIM. Nhưng U_LIM = 400 (yaw 100) lớn hơn nhiều so với công suất mixer còn lại. Giả sử hover quanh nội bộ 1100–1150, phía thấp chỉ còn khoảng 200–250 trước khi motor chạm 900. Vì vậy bộ chống windup gần như không bao giờ kích hoạt, dù motor đã bão hoà trước đó.
pid_step_climb: chỉ chặn tích phân trong khoảng 0..700, không có chống windup theo đầu ra.
Không có phản hồi bão hoà từ mixer: PID không biết esc_clamp() đã cắt đầu ra.
2. Rủi ro theo chế độ
Chế độ	Vòng rate roll/pitch/yaw	Vòng climb	Mức
ANGLE	Trung bình	Không dùng	Trung bình
ALT_HOLD	Cao (không xả tích phân trên mặt đất)	Rất cao	Cao
FAILSAFE	Như ANGLE	Rất cao (tích phân khởi đầu từ 0)	Cao
ANGLE (trung bình)
Windup trên mặt đất khi ga thấp nhưng chưa rời đất. pid_bleed_integrators() chỉ chạy khi u_throttle < 830, tức tay ga rc < ~1107. Khi tay ga lên nhẹ, u_throttle vào khoảng 830–1000, motor quay nhưng chưa nhấc máy. Nếu mặt đất nghiêng, hoặc offset/trim lệch 2–3°, thì rate_sp = 10 × lệch góc ≈ 20–30 dps. Với KI = 1.4, tích phân tăng khoảng 28–42/s và chạm I_LIM = 200 sau chừng 5–7 s. Lúc cất cánh, drone nhận một lệnh lệch rất lớn về một phía. Nó dễ gây nghiêng và trôi ngay lúc cất cánh.
Tích phân tích luỹ khi động tác nhanh rồi tiếp tục đẩy sau khi thả cần. Khi gạt cần nhanh, sai số rate lớn trong vài trăm ms (ví dụ 100 dps trong 0.3 s ≈ +40 tích phân). Tích phân này đẩy tiếp theo hướng vừa gạt sau khi bạn thả cần, và xả với hằng số thời gian khoảng 0.4 s. Đây là một phần của hiện tượng “trôi về phía vừa bay”, cộng với vấn đề accel đã nêu ở lượt trước. Mức độ vừa phải, nhưng nó đang chồng lên các nguyên nhân kia.
Bão hoà motor. Ga tối đa ở ANGLE là 1507 (rc × 0.75), chỉ còn dư khoảng 93 so với ESC_MAX = 1600. Đẩy ga cao mà vẫn sửa nghiêng thì motor chạm trần, nhưng tích phân vẫn tăng.
Chi tiết nhỏ. Điều kiện u_throttle < 830 dùng giá trị u_throttle của vòng trước vì mode_angle() gọi sau đó. Lệch một vòng 2 ms nên không đáng kể.
ALT_HOLD (cao)
Vòng climb windup ngay trên mặt đất. Đây là rủi ro lớn nhất trong toàn bộ code. Khi tay ga đẩy lên mà drone chưa rời đất, vz = 0 nên sai số = climb_sp. Với đẩy cần hết cỡ, sai số 100 cm/s làm tích phân tăng 1500/s, chạm 700 sau chưa đến 0.5 s. Khi đó u_throttle = 800 + 200 + 700 = 1700, bị cắt ở 1600 nên drone bốc lên với ga tối đa. Ga hover chỉ cần khoảng 300–350, nên I_LIM_CLIMB = 700 gấp đôi mức cần. Sau khi lên, tích phân chỉ giảm khi vz vượt climb_sp, nên drone vọt quá đà.
Không bumpless khi chuyển ANGLE → ALT_HOLD. pid_climb_st.integral bằng 0 vì ANGLE không gọi pid_climb. Ngay khi gạt công tắc, u_throttle = 800 + P ≈ 800–900, tức tụt về ga rỗng. Drone rơi, sai số dương lớn làm tích phân tăng vọt rồi vọt lên lại. Đây là kiểu dao động hạ, rồi vọt lên.
Không xả tích phân roll/pitch/yaw trên mặt đất. pid_bleed_integrators() chỉ nằm trong case MODE_ANGLE. Nếu arm ở nấc ALT_HOLD và đứng trên đất, tích phân rate cũng chạm 200 như mục ANGLE-1 nhưng không bao giờ được xả.
Đây thực chất là giữ vận tốc, không phải giữ độ cao. Code không có vòng vị trí. Khi KF bị trôi vz (đúng như comment trong altitude_kf.ino cảnh báo), tích phân sẽ đẩy ga cho tới khi vz ước lượng khớp, còn vận tốc thật sẽ bằng đúng sai số trôi đó. Không phải windup, nhưng triệu chứng giống: drone từ từ lên hoặc xuống.
Baro thường nhiễu theo luồng gió cánh quạt khi ga thay đổi. Nhiễu này đi vào vz và tích phân.
FAILSAFE (cao)
Tích phân climb bắt đầu từ giá trị cũ, thường bằng 0 nếu bạn đang bay ANGLE. Lúc mất sóng, u_throttle tụt xuống khoảng 800–900 (rơi tự do), vz âm lớn nên sai số dương lớn. Tích phân chạm 700 trong khoảng 0.2 s, và drone vọt lên với ga cao. Kết quả là rơi rồi vọt lên rồi dao động. Hiện tượng này đúng vào lúc cần hạ êm nhất, và nên sửa dù bạn chưa gặp.

3. Cách sửa (thứ tự ưu tiên)
Các hàm trong pid.ino được Arduino tạo prototype tự động, nhưng biến toàn cục thì không. Vì file nạp theo thứ tự control_task.ino trước pid.ino, tôi khai báo biến cờ trong control_task.ino và thêm hàm để control_task không phải đụng biến trong pid.ino.

3.1. Climb: chống windup theo ga thật và khởi tạo bumpless (pid.ino)

cpp
float pid_step_climb(PidState &st, float error, float kp, float ki, float kd,
                     float i_limit, float p_limit) {
  float p_term = kp * error;
  if      (p_term >  p_limit) p_term =  p_limit;
  else if (p_term < -p_limit) p_term = -p_limit;

  float d_term = kd * (error - st.prev_error) / DT_CTRL;
  float di     = ki * (error + st.prev_error) * DT_CTRL / 2.0f;

  float i_new = st.integral + di;
  if      (i_new > i_limit) i_new = i_limit;
  else if (i_new < 0)       i_new = 0;

  // Ga thực tế = ESC_IDLE + u. Chạm trần/đáy ESC thì không tích thêm.
  float u_pre = p_term + i_new + d_term;
  bool sat_hi = (u_pre > (ESC_MAX - ESC_IDLE))       && (di > 0);
  bool sat_lo = (u_pre < (ESC_MIN_ARMED - ESC_IDLE)) && (di < 0);
  if (!sat_hi && !sat_lo) st.integral = i_new;

  st.prev_error = error;
  return p_term + st.integral + d_term;
}

// Đặt sẵn tích phân bằng ga hiện tại khi vào ALT_HOLD / FAILSAFE
void pid_climb_preset(float integral) {
  pid_climb_st.integral   = constrain(integral, 0.0f, I_LIM_CLIMB);
  pid_climb_st.prev_error = 0;
}
3.2. Phản hồi bão hoà từ mixer cho vòng rate

Trong control_task.ino, thêm biến toàn cục gần đầu file:

cpp
bool pid_freeze_i = false;   // true khi mixer đang chạm giới hạn
Ngay trước dòng esc_clamp(esc_lim_lo, esc_lim_hi); thêm:

cpp
pid_freeze_i = armed && (
    esc_1 < esc_lim_lo || esc_1 > esc_lim_hi ||
    esc_2 < esc_lim_lo || esc_2 > esc_lim_hi ||
    esc_3 < esc_lim_lo || esc_3 > esc_lim_hi ||
    esc_4 < esc_lim_lo || esc_4 > esc_lim_hi);
Trong pid_step (pid.ino), sửa điều kiện chấp nhận tích phân:

cpp
bool shrinking = fabsf(integral_new) < fabsf(st.integral);
if (!(saturated_pos && windup_pos) && !(saturated_neg && windup_neg) &&
    (!pid_freeze_i || shrinking)) {
  st.integral = integral_new;
}
Khi motor đang bão hoà, tích phân chỉ được phép giảm, không được tăng. Trễ một vòng 2 ms là chấp nhận được.

3.3. Xả tích phân trên mặt đất cho mọi chế độ, và vào ALT_HOLD/FAILSAFE kiểu bumpless

Thêm #define THR_GROUND_MAX 900 và #define ALT_HOLD_MIN_ENGAGE_CM 50.0f vào khối cấu hình ở file .ino chính. Trong control_task.ino, thay khối chọn chế độ:

cpp
      int req_mode;
      if      (rc_ch[CH_MODE] == 0) req_mode = MODE_FOR_SW_0;
      else if (rc_ch[CH_MODE] == 1) req_mode = MODE_FOR_SW_1;
      else                          req_mode = MODE_FOR_SW_2;

      // Chỉ cho vào ALT_HOLD khi đã bay lên (cất cánh ở ANGLE trước)
      if (req_mode == MODE_ALT_HOLD && flight_mode != MODE_ALT_HOLD) {
        if (!sensor_present.baro || alt_kf_get_altitude_cm() < ALT_HOLD_MIN_ENGAGE_CM)
          req_mode = MODE_ANGLE;
        else
          pid_climb_preset(u_throttle - ESC_IDLE);   // bumpless
      }
      flight_mode = req_mode;
Trong nhánh failsafe, thêm preset ngay lần đầu mất sóng:

cpp
    } else if (armed && sensor_present.baro) {
      if (rc_ok_prev) {
        failsafe_start_ms = millis();
        pid_climb_preset(u_throttle - ESC_IDLE);   // tránh rơi tự do rồi vọt lên
      }
      flight_mode = MODE_FAILSAFE;
Tách việc xả tích phân ra dùng chung, và đổi ngưỡng từ 830 lên 900:

cpp
    // trước switch (flight_mode):
    if (armed && u_throttle < THR_GROUND_MAX && flight_mode != MODE_ALT_HOLD)
      pid_bleed_integrators();
và bỏ dòng if (u_throttle < 830) pid_bleed_integrators(); trong case MODE_ANGLE. Do ALT_HOLD không còn được vào khi đang ở đất, nên không cần ngưỡng riêng cho nó. Ngưỡng 900 là mức an toàn thấp nhất (dưới đó motor đang ở rỗng). Nâng dần lên khoảng 1000 sau khi bay thử tốt hơn nữa thì càng giảm windup ở đất, nhưng đánh đổi là mất trim nếu hạ ga thấp khi vẫn đang bay.

3.4. Hạ giới hạn tích phân cho khớp công suất thật (khối cấu hình)

cpp
const float I_LIM_RATE   = 80.0f,   U_LIM_RATE   = 400.0f;   // trước: 200
const float I_LIM_YAW    = 40.0f,   U_LIM_YAW    = 100.0f;   // trước: 100
const float I_LIM_CLIMB  = 450.0f,  P_LIM_CLIMB  = 200.0f;   // trước: 700
Đây là điểm xuất phát chứ không phải giá trị chắc chắn. KI_RATE_YAW = 10 rất cao nên yaw dễ chạm trần chỉ sau khoảng 1 s với sai số 10 dps. Bay thử có buộc dây, nếu thấy drone mất khả năng “giữ nghiêng” khi gió hay trim lệch thì tăng I_LIM_RATE từng bước.

3.5. Làm sau, không bắt buộc

Hệ số “I-term relax”: giảm tích phân rate khi |rate_sp| lớn (ví dụ > 60 dps), để giảm hiệu ứng đẩy tiếp sau khi thả cần.
Thêm vòng giữ độ cao thật (P trên độ cao, đầu ra là climb_sp) nếu muốn “ALT HOLD” đúng nghĩa.
4. Đo lại để chọn giới hạn chính xác
Khi bay thử ALT_HOLD, in pid_climb_st.integral lúc hover ổn định. Đặt I_LIM_CLIMB khoảng 1.3–1.5 lần giá trị đó. Tương tự, in giá trị tích phân rate roll/pitch lúc bay bình thường để chọn I_LIM_RATE. Các con số 80/40/450 ở trên chỉ nên dùng khi chưa có số liệu.

Nếu bạn gửi cho tôi log tích phân hoặc DEBUG_RATE khi bay, tôi sẽ đọc tiếp và chỉnh giới hạn cụ thể hơn.

khi bắt đầu bay ấy, đặt drone trên mặt đất, disarmed, khi bật armed mà chưa bay vội thì có rủi ro windup không trong trường hợp mặt đất có nghiêng

Soạn mã sửa lỗi cho chế độ ANGLE và giữ độ cao.
Có rủi ro, nhưng chỉ ở một số tình huống cụ thể. Trước hết tôi phải sửa số liệu trong câu trả lời trước. Tôi đã dùng hệ số ga 0.75 của bản DroneFC_CRSF_DShot. Bản ManualAcc bạn đang chạy dùng rc_ch[CH_THROTTLE] * 0.6f. Vì vậy:

u_throttle trong ANGLE chạy từ khoảng 594 (cần ga đáy) đến 1206 (hết ga), không phải 742–1507. Hết ga còn dư khoảng 394 so với ESC_MAX, chứ không phải 93.
Ngưỡng u_throttle < 830 để xả tích phân ứng với tay ga rc < ~1383, không phải 1107.
Motor chỉ vượt mức rỗng 900 khi tay ga rc ≥ 1500.
Kết luận tổng quát (windup ở đất, nên xả tích phân cho mọi chế độ) vẫn đúng, nhưng mức độ nhẹ hơn tôi nói ở một số chỗ.

1. Arm ở ANGLE, cần ga đáy: gần như an toàn
Trước khi arm, pid_reset_all() chạy mỗi vòng nên tích phân bằng 0 đúng lúc arm. prev_error vẫn được cập nhật mỗi vòng, nên không có cú D-kick.
Sau khi arm, u_throttle ≈ 594 < 830, nên pid_bleed_integrators() nhân tích phân với 0.9 mỗi 2 ms. Giả sử dốc 5°: rate_sp = 50 dps, mỗi vòng tích phân tăng 1.4 × 50 × 0.002 = 0.14. Cân bằng ở khoảng 9 × 0.14 ≈ 1.3, coi như bằng 0.
Thành phần P có u_roll ≈ 25, nhưng 594 ± 25 vẫn dưới 900. Cả 4 motor bị esc_clamp ép về 900, không có chênh lệch lực. Dù dốc 30° (u_roll ≈ 150), các motor vẫn nằm dưới 900.
Bạn đứng yên sau khi arm thì không có gì xảy ra.

2. Ba tình huống có rủi ro
A. Cửa sổ giữa lúc ngừng xả tích phân và lúc rời đất (rủi ro chính). Từ rc ≈ 1383 trở lên không còn xả, nhưng drone chưa nhấc, vì motor chưa đủ lực. Tôi ước lượng drone rời đất khoảng rc 1650–1750, nhưng bạn cần đo lại con số này. Trên dốc 5°, tích phân tăng khoảng 70/s, tức 35 sau 0.5 s, 70 sau 1 s, chạm I_LIM = 200 sau khoảng 3 s. Mức 35–70 tương đương P của 70–140 dps sai số rate, lúc nhấc máy drone sẽ bật về phía cân bằng rồi vọt quá. Cất cánh nhanh thì nhẹ, do dự thì nặng.

B. Arm khi công tắc mode đang ở nấc ALT_HOLD. Code arm không kiểm tra mode, còn pid_bleed_integrators() chỉ nằm trong case MODE_ANGLE. Vì vậy trên mặt dốc, tích phân roll/pitch tăng ngay từ lúc arm, kể cả khi cần ga ở đáy, và chạm 200 sau khoảng 3 s. Động cơ vẫn chạy rỗng (u_throttle ≈ 600), nên drone nằm yên, nhưng tích phân đã đầy. Đây là trường hợp rõ nhất của câu hỏi bạn đặt ra. Vòng climb thì ở đáy ga vẫn an toàn (tích phân kẹp ở 0). Nó chỉ nguy hiểm khi đẩy ga lên lúc chưa rời đất, như tôi đã phân tích ở lượt trước.

C. Tích phân yaw, nặng hơn bạn nghĩ. KI_RATE_YAW = 10 rất lớn, và code không có vùng chết cho cần. Lệch tâm yaw 10 µs cho yaw_rate_sp ≈ 2 dps, tích phân tăng khoảng 20/s và chạm I_LIM_YAW = 100 sau 5 s, tức u_yaw có thể lên ±100. Trên mặt đất drone bị ma sát giữ nên không xoay, sai số không bao giờ triệt tiêu. Trong ANGLE thì bị xả khi ga thấp, nhưng trong cửa sổ ở mục A và trong ALT_HOLD thì tích luỹ. Vùng chết cần gạt (mục 3.1 ở lượt đầu) giải quyết phần lớn trường hợp này.

3. Cách sửa
Thêm vào khối cấu hình:

cpp
#define THR_I_ENABLE   1000   // u_throttle dưới mức này = coi như còn ở đất
Trong control_task.ino, đặt ngay trước switch (flight_mode):

cpp
    if (armed && flight_mode != MODE_FAILSAFE && u_throttle < THR_I_ENABLE)
      pid_bleed_integrators();
Đồng thời xoá dòng if (u_throttle < 830) pid_bleed_integrators(); trong case MODE_ANGLE. Cách này xả tích phân cho cả ANGLE lẫn ALT_HOLD, và dời ngưỡng lên khoảng giữa dải ga (rc ≈ 1667), gần điểm rời đất hơn. Với ALT_HOLD trên đất, u_throttle ≈ 600 nên cũng được xả. Ngưỡng 1000 chỉ là điểm xuất phát, vì tôi chưa biết ga hover thật của bạn.

Cách chọn ngưỡng: bay thử có buộc dây, in u_throttle lúc hover, rồi đặt THR_I_ENABLE khoảng 85–90% giá trị đó. Quá thấp thì cửa sổ A vẫn còn. Quá cao thì khi bạn hạ ga dưới mức hover mà vẫn đang bay, tích phân (trim) bị xả mất.

Ngoài ra, cân nhắc chặn arm khi công tắc mode không ở nấc ANGLE, để tránh trường hợp B từ gốc. Điều kiện arm hiện tại là arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1150, bạn có thể thêm && rc_ch[CH_MODE] != 2.

Nếu đặt drone trên dốc, bật DEBUG_RATE rồi đẩy ga từ từ lên, bạn sẽ thấy rate_sp giữ ở mức khác 0 và drone không phản ứng. Gửi log đó cho tôi thì tôi tính được tốc độ tích luỹ thực tế của bạn.