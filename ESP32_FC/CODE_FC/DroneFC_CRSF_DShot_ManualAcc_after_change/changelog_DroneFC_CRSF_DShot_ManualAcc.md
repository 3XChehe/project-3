# Changelog — ManualAcc after_change

Đối chiếu với ManualAcc gốc bên cạnh; **original chỉ đọc, không thay đổi**. Bản ghi hiện hành ngày **2026-10-16**. Kết quả test tập trung trong [báo cáo bốn mục](BAO_CAO_XU_LY_MANUALACC.md); changelog không chứng minh an toàn bay.

## 2026-10-16 — chỉnh trực tiếp after_change theo nền ManualAcc gốc

Không giữ nguyên after_change cũ rồi chỉ thêm test. Controller được viết lại tại thư mục này; cấu hình/filter được đối chiếu và đưa về chuẩn original. Những helper PID/driver safety phù hợp từ target cũ được giữ có chủ đích. Không copy đè toàn bộ thư mục original sang target và không tạo phiên bản firmware hoạt động thứ ba.

| Cấu hình | Bản hiện tại |
|---|---|
| Công tắc | SW0/SW1 ANGLE, SW2 ALT_HOLD như original. FAILSAFE tự động do mất RC, không gán công tắc. |
| Gain | Climb Kp/Ki/Kd=3/15/0, không phải Ki4 của after_change cũ; rate/góc/yaw và các giới hạn giữ original. |
| IMU filter | R_acc=9 cố định như original; bỏ dynamic trust theo lệnh/norm. Giữ numeric guard và atan2 không chia0. |
| Pipeline | Giữ sensor→ước lượng→PID cascade→mixer→DShot, chân/sign, offsets, DLPF, ANGLE RC×0.6, DT2ms, KF dọc Q/R/cadence500Hz. |
| Debug/startup | DEBUG_ACC_OFFSET mặc định, có thể chọn macro khác mà không redefine. Target bỏ khối tự quay motor; original không bị sửa. Gyro startup500×2ms, không tracking bias disarmed. |

### Anti-windup và vòng đời

- Disarm reset toàn bộ PID/history/output, setpoint LPF, phase/evidence/ramp/limiter, gửi800→DShot0 rồi return trước PID.
- ARM chỉ theo frame kênh RC thực còn mới, cạnh OFF→ON/ga<1150; không tự arm ở boot/reconnect khi ARM đã ON. ARM ALT cần readiness tĩnh1s; từ chối phải OFF→ON lại.
- ALT trực tiếp: WAIT giữ motor900/không PID/I → ga>1580 liên tục200ms và readiness đạt mới TAKEOFF → ramp tương đối/attitude PD/I0 → baro tăng>30cm/trend dương được xác nhận300ms trên mẫu mới mới FLIGHT/seed PI.
- Evidence theo cả ANGLE và dùng chung phiên ARM. Mọi ANGLE→ALT chưa có evidence đều bị từ chối, kể cả phiên bắt đầu từ ALT. Đổi MODE không reset hoặc tự nâng phase.
- Không tự reset I/quay WAIT/disarm vì ga thấp/vz≈0 sau FLIGHT. Bleed ANGLE chỉ khi chưa có evidence và xét ga hiện tại, không ga của tick trước. Sau đặt xuống cần ARM OFF trước lần thử tiếp theo, không cần reset ESP32.
- Tách nguồn DIRECT/RAMP/CLIMB_PI. Nguồn dọc slew+80/−150 đơn vị/s, chặn900–1600; không thêm slew từng motor.
- Giữ own-output conditional integration và feedback clipping riêng từng rate-axis. Climb nhận collective clamp/slew + motor-ceiling residual; motor-floor do rate không cản giảm climb I.
- Climb giữ phần bước I khả thi để sai số bền vững vẫn tiến qua slew; remix cùng mốc đầu tick và commit limiter một lần.
- Seed I=clamp(T_applied−800−P_current,0,700), dùng setpoint mới, không tích thêm I ở tick seed. Seed từ average lệnh motor nguyên nhưng giữ mốc float limiter RAMP→PI; DIRECT→PI mới đồng bộ mốc từ actual applied.

### RC/cảm biến và fallback

- Giữ raw RC riêng công tắc decode; hụt mutex không decode0/1/2 lần nữa. Timestamp chỉ từ frame kênh thực, không từ cache/link-statistics; RC/baro source age≤200ms, wrap-safe.
- Readiness dùng lịch sử1s/cửa sổ500ms, tilt<10°, mỗi gyro rate<5°/s, |mean a_z|<0.01g, baro peak-to-peak<20cm. Chỉ mẫu baro mới đóng góp baro history/trend. Không phải detector grounded/airborne chắc chắn.
- ALT-WAIT mất RC disarm. TAKEOFF mất RC trước evidence chuyển DEGRADED, giữ collective/PD/I0; latch launch_aborted đến disarm, không tăng lại sau RC hồi.
- Chỉ confirmed flight/dữ liệu dọc hợp lệ/chưa bị khóa mới vào FAILSAFE−60cm/s khi mất RC; các trường hợp còn lại DEGRADED.
- Fault/stale/KF failure khóa PI; reset/reacquire sau khi source không còn PI, tích history mới và thao tác MODE ANGLE→ALT sau hồi phục. Fault dọc khi ANGLE đã có evidence cũng latch inhibition: không tự mở FAILSAFE sau khi baro hồi trong lúc RC vẫn mất. ANGLE đang hoạt động có thể giữ điều khiển thủ công.
- RC hồi phải đổi nấc MODE sau lúc hồi mới tiếp quản; trước xác nhận cả ga cao/thấp đều không điều chỉnh collective. DEGRADED sau tiếp quản cho relative collective; trước FLIGHT chỉ giữ/giảm, WAIT giữ900/không torque. ARM OFF vẫn ưu tiên.
- Giữ strict ID, baro DRDY/finite/áp suất>0/đủ50 mẫu reference, fault khác no-new; IMU/KF cập nhật transactional, chặn numeric overflow. IMU/control-output hard fault latch disarm, không gọi đó là hạ an toàn.
- Log phase/history_ok/reason/ready/takeover/denied/I/collective. Không Serial trong control500Hz. history_ok chỉ là đủ lịch sử dữ liệu hợp lệ, không phải chứng nhận quality vật lý.

### Cơ chế cũ không còn hoạt động

Đã bỏ dynamic accel trust/R=9/trust², baro jump/innovation quarantine, mismatch vận tốc KF–trend và LED trạng thái runtime. Không thay các detector này bằng một claim rằng finite/freshness/history phát hiện hết lỗi. Bias hữu hạn, áp suất do luồng cánh và dữ liệu đóng băng vẫn là giới hạn.

Tài liệu công thức/physics được cập nhật Ki15, mapping mới và cơ chế đang có; không còn mô tả SW2 là FAILSAFE hoặc ANGLE→ALT trên nền là được phép.

### Kiểm chứng bản hiện tại

Host runner kiểm mã .ino/BLA thật với stub phần cứng; kết quả cuối trong [báo cáo](BAO_CAO_XU_LY_MANUALACC.md). Bộ test được điều chỉnh có chủ đích, không giữ số lượng/test cho subsystem đã loại để tạo ấn tượng cùng phạm vi. Chưa build/upload target ESP32 hoặc thử motor/bay.

## Lịch sử before-change và backup

Trước lần sửa này, after_change dùng SW0 ANGLE/SW1 ALT/SW2 FAILSAFE, Ki_climb4, dynamic trust và quality subsystem mở rộng/LED. Baseline cũ được chạy trước sửa: 67/67 host, 7/7 macro compile, 10/10 original reference hash. **Đó không phải kết quả bản hiện tại.**

[Backup ngày 2026-10-16](../../../backups/manualacc_after_change_20261016/manifest.json) giữ nguyên firmware/tài liệu và tests cũ với checksum22 file, đồng thời ghi checksum11 file original. Các mô tả/quy tắc cũ trong backup chỉ phục vụ đối chiếu/khôi phục, không phải hướng dẫn vận hành target mới.

Những test chỉ cho trust scheduling, LED pattern, jump/innovation quarantine, mismatch detector đã được loại khỏi suite hiện hành vì không còn production code tương ứng; retained numeric/driver/RC/PID/mixer coverage không bị thay bằng oracle dễ hơn. Các profile airborne phải kiếm evidence qua control_step thật, không ép phase FLIGHT để pass.

## Giới hạn không đổi

Anti-windup chỉ phản hồi giới hạn lệnh, không đo lực đẩy/RPM/tiếp đất. Sensor profile tổng hợp không phải mô hình motor–khí động vòng kín. Giữ gain gốc không chứng minh gain phù hợp phần cứng. Mất cả RC/baro hoặc hard IMU fault không có bảo đảm hạ an toàn; motor có thể tiếp tục quay khi đặt xuống nếu chưa disarm.
