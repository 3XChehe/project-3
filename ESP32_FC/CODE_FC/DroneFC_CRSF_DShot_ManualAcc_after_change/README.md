# DroneFC_CRSF_DShot_ManualAcc_after_change

Firmware thử nghiệm ESP32 WROOM32, ICM-20602/SPI, BMP388/I2C, RP3 ELRS/CRSF và GOKU G55M DShot600/RMT. Cập nhật **2026-10-16**: sửa trực tiếp thư mục này, lấy cấu hình và luồng ManualAcc gốc làm chuẩn; ManualAcc gốc chỉ đọc, giữ nguyên.

**Chưa được xác nhận đủ an toàn để bay.** Host tests không kiểm chứng lực đẩy, dấu trục, dây/chiều motor, nhiễu thực hoặc deadline 500Hz. Tháo toàn bộ cánh trước kiểm tra bàn. Test PASS không phải giấy phép thử bay.

## Sketch và cấu hình gốc

- Mở [DroneFC_CRSF_DShot_ManualAcc_after_change.ino](DroneFC_CRSF_DShot_ManualAcc_after_change.ino).
- Mục tiêu Arduino ESP32 core **2.0.17**, RMT legacy, thư viện **BasicLinearAlgebra**, Serial 500000baud; không mặc định tương thích core 3.x.
- **SW0/SW1 = ANGLE; SW2 = ALT_HOLD. FAILSAFE chỉ tự động khi mất RC**, không có công tắc gọi FAILSAFE thủ công.
- Giữ gain gốc: rate roll/pitch 0.5/1.4/0.03; yaw 1/10/0; góc 10/0/0; climb **3/15/0**. Không kế thừa Ki climb 4 của after_change cũ. Giữ gain không có nghĩa đã tune phù hợp phần cứng.
- Giữ chân, dấu mixer, offset accel, DLPF, DT_CTRL=0.002s, KF dọc hai trạng thái/Q/R/predict-correct 500Hz. Kalman góc trở về **R_acc cố định 9**, không có dynamic accel trust.
- DEBUG_ACC_OFFSET là mặc định nếu không chọn macro khác. CALIBRATE_ACCEL là công cụ đo offset riêng, dừng trước khi tạo task bay.

## ARM và ALT_HOLD trực tiếp

ARM cần frame kênh RC thực còn mới, cạnh OFF→ON và ga<1150. ARM trực tiếp ALT ở **SW2** còn cần readiness tĩnh. Từ chối ARM không tự chuyển thành ANGLE hoặc tự arm khi cảm biến hồi; cần OFF→ON mới. Boot/reconnect với ARM đang ON không tự arm.

**Đặt drone xuống sau một lần thử: ARM OFF trước lần thử tiếp theo; không cần reset ESP32.** Disarm xóa PID/I/lịch sử D/output, setpoint lọc, phase/evidence/ramp; gửi 800→DShot0 rồi return trước PID. Firmware không có cảm biến xác nhận tiếp đất. Nếu vẫn armed sau khi đặt xuống, I bay có thể còn.

| Pha | Hành vi |
|---|---|
| WAIT_TAKEOFF=0 | Arm trực tiếp ALT: bốn motor 900, không chạy PID/I. **Motor vẫn có thể quay**, không phải disarm. |
| TAKEOFF=1 | Ga>1580 liên tục 200ms và readiness đạt mới bắt đầu ramp. Giữa cần giữ collective, lên tăng, xuống giảm; attitude PD hoạt động, mọi I khóa 0. Không nhấc lên thì không tự bật PI theo timeout. |
| FLIGHT=2 | Baro tăng>30cm so với mốc takeoff, trend dương được xác nhận thêm 300ms trên mẫu nguồn mới: seed PI rồi mở I. Đây là heuristic, không phải xác nhận airborne chắc chắn. |

Readiness: lịch sử 1s qua cửa sổ 500ms; roll/pitch<10°, mỗi gyro rate<5°/s, `|mean(acc_earth_z)|<0.01g`, baro peak-to-peak<20cm. Baro chỉ đếm timestamp mẫu mới, không đếm snapshot lặp 500Hz. Ngưỡng chưa được hiệu chuẩn theo nhiễu/luồng cánh thực.

Bằng chứng tăng độ cao cũng được theo dõi khi ANGLE và dùng chung suốt phiên ARM. **ANGLE→ALT khi chưa có bằng chứng bay bị từ chối, giữ ANGLE**, kể cả phiên từng ARM từ ALT rồi đổi qua ANGLE. Không đổi MODE để bỏ qua bảo vệ trên nền. Khi đã có bằng chứng mới được chuyển sang ALT và seed PI.

Sau FLIGHT, ga thấp/vz≈0 không xóa I, không quay WAIT hoặc tự disarm. Climb là điều khiển **vận tốc dọc**, không có vòng giữ vị trí độ cao hoặc vị trí ngang. Cần giữa 1500, deadband 1460–1540 đặt mục tiêu 0cm/s; ngoài vùng này ánh xạ liên tục tới ±100cm/s, có lọc.

## Anti-windup và chuyển PI

- Tách nguồn collective DIRECT/RAMP/CLIMB_PI. Nguồn dọc chặn 900–1600 và slew **+80/−150 đơn vị nội bộ/s**, tương ứng +0.16/−0.30 mỗi tick 2ms. ANGLE vẫn ga trực tiếp `RC×0.6`.
- Rate nhận phản hồi clipping riêng từng trục qua inverse mixer. Chỉ từ chối bước I đẩy sâu thêm vào saturation; vẫn cho unwind. Không freeze mọi trục và không đưa residual motor vào PID góc có output °/s.
- Climb nhìn cả clamp/slew collective và phần hụt do motor chạm trần. Motor chạm sàn do rate không cản bước giảm climb I khi collective còn khả thi.
- Giữ phần bước I khả thi, không rollback toàn bộ làm collective đứng mãi dưới sai số bền vững. Remix dùng cùng mốc limiter đầu tick và chỉ commit một lần.
- Seed: `I=clamp(T_applied−800−P_current,0,700)`. Ví dụ T1200/P150 → I250, không phải 400. Tick seed không tích I thêm. Nếu seed vượt giới hạn thì báo seed_limited và vẫn slew, không hứa bumpless tuyệt đối.
- T_applied là average bốn lệnh motor đã lượng tử hóa, không phải lực đẩy đo được. Seed không xóa mốc float limiter RAMP→PI; chỉ DIRECT→PI mới đồng bộ mốc từ lệnh áp dụng.
- Slew chỉ giới hạn collective, **không giới hạn mọi bước thay đổi từng motor do P/D tư thế**. Bảo vệ không biết motor hỏng, cánh thiếu lực hoặc thân đang bị giữ.

## Mất tín hiệu, lỗi và phục hồi

| Tình huống | Hành vi |
|---|---|
| WAIT của phiên ALT mất RC | Disarm, không chuyển sang PI hạ. |
| TAKEOFF mất RC trước bằng chứng bay | DEGRADED, giữ collective/PD, I0; launch_aborted khóa đến disarm, không tiếp tục tăng sau khi RC hồi. |
| Đã xác nhận FLIGHT, mất RC, dữ liệu dọc hợp lệ và chưa bị khóa | FAILSAFE đặt −60cm/s, tư thế phẳng/yaw rate0, cùng seed/anti-windup/slew. Không bảo đảm hạ an toàn. |
| Dữ liệu dọc stale/lỗi trong mode dọc | Khóa PI, DEGRADED. Giữ collective cuối; còn RC đã tiếp quản thì điều chỉnh tương đối. Không còn giữ tốc độ/độ cao. |
| Lỗi dọc lúc đang ANGLE đã có bằng chứng bay | Có thể giữ ANGLE thủ công, nhưng khóa quyền vào PI; mất RC sau đó không tự mở PI chỉ vì baro hồi. |
| DEGRADED trước FLIGHT | Chỉ giữ/giảm collective khi pilot đã tiếp quản, không tăng; riêng WAIT giữ 900/không torque. Muốn thử cất cánh lại phải disarm. |
| Mất cả RC và dữ liệu dọc | Giữ collective/cân bằng nếu IMU còn dùng được; không bảo đảm hạ, giữ cao hay tắt motor. |
| IMU/control output không hữu hạn | Latch control_fault và disarm ngay, không qua slew; không phải hạ an toàn. |

RC/baro có tuổi nguồn tối đa 200ms; hụt mutex hoặc đọc cache không làm mới timestamp. Lỗi dọc yêu cầu reset/reacquire KF **sau khi nguồn PI đã khóa**, tích lại lịch sử 1s, rồi xác nhận MODE ANGLE→ALT; không tự mở PI lúc baro hồi. Vz0 khi reset chỉ là khởi tạo, không chứng minh đang đứng yên.

RC vừa hồi phải đổi nấc MODE khác nấc ghi nhận lúc hồi để tiếp quản. **Trước xác nhận, cần ga cao hay thấp đều không điều chỉnh collective**; sau xác nhận mới điều chỉnh tương đối trong DEGRADED. ARM OFF luôn ưu tiên với frame hợp lệ. Không tự disarm theo ga thấp/vz≈0 sau 2s; motor có thể tiếp tục quay trên đất.

## Phạm vi cảm biến và những cơ chế không mang sang

Bản này **không có** quarantine baro jump/innovation, kiểm mismatch vận tốc KF–trend, dynamic accel trust hoặc LED trạng thái runtime của after_change cũ. Kiểm finite/freshness/history không chứng minh số đo hữu hạn là đúng; bias, áp suất do luồng cánh hoặc cảm biến đóng băng vẫn có thể lọt và làm PI đổi ga sai.

Giữ kiểm ID ICM-20602=0x12/BMP388=0x50, chặn mẫu lỗi/áp suất≤0, yêu cầu đủ 50 mẫu tham chiếu baro; phân biệt no-new và fault. Baro init lỗi không tự runtime reinit. Gyro calibration startup 500×2ms mỗi đợt; không tự học bias lúc disarmed, chưa bù drift nhiệt online. Phải giữ đứng yên khi calibration.

Thang nội bộ: 800→DShot0; armed 900→48, 1600→2047. Không phải µs/RPM/Newton/%thrust. RC990–2010 trong ANGLE cho collective594–1206 trước mixer/clamp; chưa xác nhận dải đó đủ lực đẩy.

## Log và chân kết nối

DEBUG_ALTITUDE/DEBUG_RC có phase, **history_ok**, reason, ready, takeover, denied, I bốn vòng và collective requested/limited, qua snapshot khoảng 5Hz. `history_ok` chỉ là đủ lịch sử dữ liệu hợp lệ, không phải quality detector vật lý. Reason: 0 acquiring, 1 OK, 2 stale/driver-invalid, 3 KF numeric, 4 chưa đạt readiness tĩnh. Reason4 không tự khóa PI đang FLIGHT. Chỉ chọn một macro debug khi đo tải.

LED14 phục vụ calibration/startup; không có pattern WAIT/DEGRADED runtime và không có cảnh báo gửi lên tay điều khiển.

| Tín hiệu | GPIO | Vị trí/chiều khai báo, nhìn từ trên |
|---|---:|---|
| M1 | 27 | Sau phải, CW |
| M2 | 26 | Trước phải, CCW |
| M3 | 25 | Sau trái, CCW |
| M4 | 33 | Trước trái, CW |
| IMU CS/SCK/MOSI/MISO | 5/18/23/19 | SPI |
| LED | 14 | Calibration/startup |
| CRSF RX/TX | 16/17 | RP3 TX/RX |
| BMP388 SDA/SCL | 21/22 | I2C 0x76 |

Khối tự quay từng motor lúc startup của original đã được bỏ **trong target này**; original giữ nguyên. Kiểm dấu trục/dây/chiều motor cần tháo cánh.

Hiệu chỉnh accel: bật CALIBRATE_ACCEL, đặt phẳng/đứng yên, lấy 2000×2ms mẫu, ghi ACC_OFFSET_*_G vào file chính rồi tắt macro. DEBUG_ACC_OFFSET kiểm tĩnh X≈0/Y≈0/Z≈1, không xác nhận rung khi motor chạy.

## Kiểm thử, backup và giải thích

- Chạy từ gốc workspace: `python Drone/tests/manualacc/run_tests.py --debug-check`.
- Dùng firmware và BLA thật với stub phần cứng, không phải ESP32 target build hoặc mô hình bay kín.
- [Báo cáo bốn mục và kết quả kiểm chứng](BAO_CAO_XU_LY_MANUALACC.md).
- [Cảm biến→PID→mixer, phép tính và ví dụ vật lý](GIAI_THICH_LUONG_CAM_BIEN_PID_MIXER.md).
- [Changelog và phần khác với after_change cũ](changelog_DroneFC_CRSF_DShot_ManualAcc.md).
- [Backup firmware/tài liệu/tests cũ và checksum](../../../backups/manualacc_after_change_20261016/manifest.json). Đây chỉ là bản khôi phục, không phải firmware bàn giao. Kết quả 67 case của bản cũ không phải kết quả bản này.

## Giấy phép và ghi công

Dựa trên firmware Nguyễn Văn Quý (Kỹ Thuật UAV), PID cascade tham khảo Carbon Aeronautics; giữ **GPL-3.0** và ghi công trong mã nguồn.
