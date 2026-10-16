# Báo cáo xử lý ManualAcc after_change

Cập nhật **2026-10-16**. Sửa trực tiếp firmware trong thư mục after_change, lấy **ManualAcc gốc làm chuẩn cấu hình và luồng**, không sửa original. Chỉ thay đổi dưới Drone; không upload, chạy motor, thử bay, cài toolchain, commit hoặc push.

Mục tiêu: bổ sung anti-windup ALT mà vẫn cho **ARM/cất cánh trực tiếp ALT**, không buộc dùng ANGLE. Chưa xác định được nguyên nhân duy nhất của sự cố thực; chưa biết lần đặt xuống trước đó đã ARM OFF hay chưa.

## 1. Vấn đề

| Nhóm | Rủi ro đã xác nhận từ mã và lý do cần sửa |
|---|---|
| Khởi đầu/thử lại | ManualAcc gốc gọi climb PI khi armed ALT dù còn nằm trên nền. Sai số vận tốc không thể được đáp ứng vẫn làm I tăng. Đặt xuống mà không disarm còn có thể giữ I bay từ lần trước. Disarmed gốc reset I nhưng vẫn chạy tiếp PID trong cùng lượt. |
| Giới hạn đầu ra | Kẹp I không đủ: PID không biết motor bị clip sau mixer hoặc collective đang bị slew. Freeze mọi I làm mất khả năng unwind/điều khiển các trục chưa bão hòa. Floor do rate còn có thể bị nhầm thành thiếu collective, cản bước giảm climb I. |
| Chuyển mode/pha | Seed chỉ lấy T−800 mà không trừ P mới gây cộng thêm P vào ga. Arm ANGLE rồi đổi ALT trên nền có thể bỏ qua khóa I nếu chỉ kiểm arm trực tiếp ALT. Seed từ lệnh motor nguyên cũng có thể làm mất mốc float limiter và vượt slew khi đổi RAMP→PI. |
| Dữ liệu và phục hồi | Cache/mutex không được coi là frame mới. Baro/RC stale hoặc KF lỗi không được tiếp tục cho PI dùng trạng thái cũ. Lỗi baro trong ANGLE rồi mất RC cũng không được tự mở PI sau khi sensor hồi. Finite/freshness không phát hiện hết bias hoặc áp suất sai nhưng hữu hạn. |
| Phạm vi bản dựng | after_change cũ khác original ở mapping, Ki climb, dynamic trust và subsystem quality/LED. Giữ nguyên toàn bộ bản cũ sẽ không đúng yêu cầu xây lại theo ManualAcc gốc; tài liệu cũ còn mô tả các policy không còn áp dụng. |

Không quy mọi hiện tượng tăng ga cho Ki hoặc I. P, D, ước lượng sai, hướng mixer/motor sai hoặc thiếu lực đẩy đều có thể làm motor phản ứng mạnh. Anti-windup không phải cảm biến xác nhận tiếp đất.

## 2. Xử lý

**Nền cấu hình:** SW0/SW1 ANGLE, SW2 ALT_HOLD; FAILSAFE chỉ tự động do mất RC. Climb Kp/Ki/Kd=3/**15**/0; giữ các gain/giới hạn còn lại, ANGLE RC×0.6, chân, dấu mixer, offsets, DLPF, DT2ms và KF dọc Q/R/cadence500Hz. Kalman góc dùng lại R_acc cố định9. Không tự giảm Ki để che triệu chứng; bộ gain này chưa được xác nhận bằng bay thật.

| Nhóm xử lý | Hành vi hiện hành |
|---|---|
| Vòng đời ARM | Frame kênh RC thực, còn mới, ga<1150 và cạnh OFF→ON. Boot/reconnect với ARM ON không tự arm. ARM ALT cần readiness tĩnh1s; bị từ chối phải OFF→ON lại. Disarm reset PID/history/output, LPF, phase/evidence/ramp/limiter, gửi800→DShot0 rồi return trước PID. |
| WAIT→TAKEOFF | Arm trực tiếp ALT vào WAIT: motor900, không PID/I, motor vẫn có thể quay. Ga>1580 liên tục200ms và readiness đạt mới ramp; attitude PD hoạt động, toàn bộ I khóa0. Giữa cần giữ collective; lên/xuống thay đổi tương đối. Không tự bật PI vì giữ ga lâu. |
| TAKEOFF→FLIGHT | Baro tương đối tăng>30cm, trend dương được xác nhận300ms trên mẫu nguồn mới mới seed PI/mở I. Evidence dùng chung phiên ARM và cũng theo dõi trong ANGLE. Mọi ANGLE→ALT chưa có evidence đều bị từ chối, giữ ANGLE; đổi mode không reset hoặc tự nâng phase. |
| Anti-windup | Conditional integration theo own-output; residual inverse mixer riêng roll/pitch/yaw. Climb nhận clamp/slew collective và phần motor chạm trần; không lấy motor-floor do rate để cản giảm I. Chỉ từ chối bước I bất lợi, cho unwind; không feedback motor-unit vào PID góc có output°/s. |
| Nguồn collective | Tách DIRECT/RAMP/CLIMB_PI. Nguồn dọc chặn900–1600, +80/−150 đơn vị/s (+0.16/−0.30 mỗi2ms). Giữ phần bước climb I khả thi, remix cùng mốc đầu tick và commit limiter một lần. Seed RAMP→PI không xóa mốc float vì lượng tử hóa motor. |
| Seed PI | I_seed=clamp(T_applied−800−P_current,0,700), P dùng setpoint mới, kể cả FAILSAFE−60cm/s. Tick seed không tích I thêm. Báo seed_limited nếu ngoài miền I, vẫn slew; không hứa liên tục tuyệt đối từng motor. |
| Dữ liệu tối thiểu | RC/baro timestamp nguồn≤200ms, phép trừ wrap-safe. Baro publish DRDY hợp lệ, fault khác no-new. Kiểm ID, hữu hạn và đổi đơn vị; KF cập nhật transactional. Reacquire chỉ sau khi nguồn PI khóa, cần lịch sử mới1s trước khi đủ điều kiện PI. |
| Readiness/evidence | Cửa sổ500ms, lịch sử1s; tilt<10°, rate từng trục<5°/s, trị tuyệt đối mean gia tốc dọc<0.01g, baro peak-to-peak<20cm để arm/ramp. Chỉ mẫu baro mới đóng góp thống kê/trend. Đây là heuristic chưa hiệu chuẩn, không xác nhận chắc chắn grounded/airborne. |
| Mất RC | ALT-WAIT disarm. TAKEOFF trước evidence chuyển DEGRADED/giữ collective/PD/I0, khóa launch_aborted đến disarm; không tăng lại khi RC hồi. Đã bay và dữ liệu dọc hợp lệ/chưa bị khóa mới dùng FAILSAFE−60cm/s; nếu không DEGRADED. |
| Lỗi/phục hồi dọc | Lỗi trong mode dọc khóa PI, chuyển DEGRADED; lỗi khi ANGLE đã có evidence cũng latch inhibition, không cho tự vào FAILSAFE sau khi baro hồi. ANGLE đang hoạt động có thể giữ điều khiển thủ công. Hồi dữ liệu không tự mở PI: cần lịch sử hợp lệ rồi xác nhận MODE ANGLE→ALT. Xác nhận sớm lúc dữ liệu còn lỗi không được giữ làm quyền mở PI về sau. |
| Quyền pilot | RC hồi phải đổi MODE sau lúc hồi mới tiếp quản; trước xác nhận, cả ga cao/thấp đều không điều chỉnh collective. Sau xác nhận DEGRADED nhận điều chỉnh tương đối; trước FLIGHT chỉ giữ/giảm, WAIT giữ900/không torque. ARM OFF luôn ưu tiên với frame hợp lệ. |
| Sau FLIGHT/lỗi cứng | Không xóa I/quay WAIT/disarm vì ga thấp hoặc vz≈0. Bleed ANGLE dùng ga hiện tại và chỉ khi chưa có evidence bay. IMU/control-output không hữu hạn latch fault/disarm ngay, không qua slew; không gọi đó là hạ an toàn. |
| Startup/quan sát | Bỏ khối tự quay từng motor trong target. Giữ gyro startup500×2ms, không tracking bias lúc disarmed. Log phase/history_ok/reason/ready/takeover/denied/I/collective qua snapshot; history_ok không có nghĩa kiểm chứng chất lượng vật lý. |

Phép tính phản hồi, trước đổi motor sang int:

```text
T_lim = clamp(clamp(T_req,900,1600),
              max(900,T_prev−150×dt), min(1600,T_prev+80×dt))
d_i = clamp(mixer_i(T_lim,r,p,y),900,1600) − mixer_i(T_lim,r,p,y)
residualR = (−d1−d2+d3+d4)/4
residualP = ( d1−d2+d3−d4)/4
residualY = ( d1−d2−d3+d4)/4
residualClimb = T_lim−T_req + Σ min(0,d_i)/4
```

Nếu ΔI×residual<−0.0001, rate rollback bước I của trục đó; climb giữ phần khả thi:

```text
I_new = clamp(I_candidate + residualClimb,
              min(I_before,I_candidate), max(I_before,I_candidate))
```

**Ví dụ Ki15:** T_prev1200, P200, I_before200, sai số ổn định+100cm/s: ΔI=15×100×0.002=3, I_candidate203, T_req1203. Slew cho1200.16, residual−2.84 nên I_new200.16. Collective tiếp tục tiến0.16 thay vì rollback hết rồi đứng mãi. Feedback chỉ tác động đoạn bước I hiện tại, không tự xóa toàn bộ I cũ.

**Ví dụ seed:** T_applied1200, P_current150 ⇒ I_seed250, output1200; lấy I400 sẽ đòi1350. Nếu I cần âm hoặc>700 thì seed phải bị giới hạn; slew chỉ hạn chế collective, không triệt bước P/D tư thế.

**Không mang sang bản này:** dynamic accel trust/R=9/trust², quarantine jump/innovation baro, mismatch vận tốc KF–trend và LED trạng thái runtime. Các cơ chế đó nằm trong backup cũ, không được mô tả như đang hoạt động. Bản này không bảo đảm phát hiện mọi sai lệch hữu hạn; không thay chúng bằng một tên history_ok rồi coi là cùng mức bảo vệ.

Mã chính: [controller](control_task.ino), [PID/seed/feedback](pid.ino), [types](types.h), [KF dọc](altitude_kf.ino), [IMU](imu_icm20602.ino), [cấu hình](DroneFC_CRSF_DShot_ManualAcc_after_change.ino). PID/driver guards phù hợp từ target cũ được giữ có chủ đích; controller và cấu hình/filter được sửa trực tiếp theo baseline gốc, **không phải toàn bộ thư mục được copy đè từ original**.

## 3. Vì sao xử lý như vậy

- **Không tích I để đòi chuyển động chưa thực hiện được trên nền:** WAIT/TAKEOFF tách khỏi PI bay, nhưng không buộc người dùng cất cánh ANGLE. Evidence chung còn chặn đường đổi mode lách bảo vệ.
- **Bảo vệ đúng tầng và đúng hướng:** giới hạn riêng từng trục khác giới hạn collective; motor-floor do moment không đồng nghĩa ga chung quá thấp. Cho unwind và giữ phần bước I khả thi tránh cả windup lẫn PI bị đứng do chính slew.
- **Chuyển PI từ lệnh đã áp dụng, không từ I cũ:** phải trừ P mới; không tích thêm I ngay tick seed. Mốc limiter float phải độc lập với việc lượng tử hóa motor để giữ giới hạn tốc độ.
- **Không tự khôi phục quyền bằng cache/sensor hồi:** xác nhận history và thao tác MODE phải có chủ ý; lỗi dọc khi bay ANGLE cũng không được quên trước khi mất RC. RC hồi với cần ga cao không phải yêu cầu tiếp tục cất cánh.
- **Không đoán touchdown:** ga thấp/vz≈0 cũng xuất hiện khi hạ hoặc hover. Reset I/cắt motor theo chúng có thể phá điều khiển trong không trung; đổi lại pilot phải ARM OFF sau khi đặt xuống.
- **Giữ ranh giới sửa logic và tuning:** lấy gain/filter gốc làm chuẩn, không đồng thời tune estimator/thrust/gain và không tuyên bố các heuristic chưa đo là cơ chế phát hiện lỗi chắc chắn.

## 4. Kết quả

Đã chạy lại độc lập trên firmware và tests hiện tại; kết quả 67/67 của bản cũ bên dưới **không phải** kết quả bản dựng mới. Lệnh kiểm chứng:

```text
python Drone/tests/manualacc/run_tests.py --debug-check
```

Môi trường đã kiểm: Windows, MSYS2 UCRT64 **g++14.2.0 Rev2**, C++11/Wall/Wextra/Werror; **BasicLinearAlgebra5.1 thật** tại thư viện Arduino đã cài. Runner dùng `.ino` thật, stub Arduino/FreeRTOS/SPI/Wire/UART/RMT; ghép primary/tab alphabet và mô phỏng prototype. Không phải Arduino preprocessor hoặc ESP32 SDK; mỗi case chạy process riêng.

| Kiểm chứng bản mới | Kết quả thực chạy |
|---|---|
| Tổng host regression | **70/70 PASS**. Không dùng số67 của baseline cũ làm kết quả mới. |
| Debug/calibration | **7/7 compile PASS**: DEBUG_ATTITUDE/RATE/ALTITUDE/RC/ACC_OFFSET/LOOP_TIME và CALIBRATE_ACCEL. Calibration chỉ compile, không chạy nhánh setup phần cứng. |
| Reference baseline | **10/10 SHA-256 không đổi** trong runner; kiểm manifest riêng mở rộng thành11/11 gồm transcript Sonnet. |
| Ground/lifecycle | WAIT30s có lệnh nghiêng/yaw vẫn motor900/I0; debounce, ramp tăng/giữ/giảm/I0; re-arm không reset ESP32 không mang state/LPF cũ; readiness không đủ không tự arm. |
| PI/mixer/slew | Seed P dương/âm/limited và tick không tích thêm; FS tự động dùng setpoint−60; selective rollback/mixed axes/unwind hai chiều; floor-rate không giữ climb I; sai số bền vững200tick vẫn tiến qua slew. |
| Launch/mode | ANGLE trên nền không vào ALT; ANGLE có evidence mới seed. Evidence dựa timestamp baro mới, mode toggle không lách phase. Profile IMU/baro nhất quán50Hz kiếm FLIGHT qua control_step thật, rồi leo/coast/hover/hạ; không ép FLIGHT trong fixture. |
| RC/fault/recovery | WAIT mất RC disarm; TAKEOFF loss khóa abort và không tăng lại; baro fault trong ANGLE hoặc mất RC+baro đồng thời không tự mở PI sau hồi; fault ngắn100ms/gap dài cần reacquire/history và xác nhận mới; RC high/low đều không takeover khi chưa đổi MODE. |
| Regression nền | CRSF CRC/first-frame/mutex/timestamp/rollover; disarm/PID reset/NaN/Inf; baro ID/finite/fault/no-new/reference; KF numerical overflow/transactional; DShot bounds/CRC; telemetry và fixed-R IMU đều PASS. Test esc_init qua RMT stub kiểm đủ1.240 gói startup: tất cả throttle0/disarm, không có lệnh motor-test bị che bởi gói disarm cuối. |

Bộ test đã tái hiện lỗi seed RAMP→PI: limited trước1064.47 nhưng applied nguyên1064; nếu gán lại mốc limiter từ applied sẽ giảm0.47 trong2ms, vượt giới hạn0.30. Đã sửa giữ mốc limiter cũ khi source đã limited; oracle slew nghiêm ngặt giữ nguyên và PASS. Hai regression khác kiểm I ANGLE không bleed sau evidence và lỗi dọc trong ANGLE không quên latch khi RC mất.

Những test chỉ thuộc dynamic trust/LED/jump/innovation/mismatch của after_change cũ không còn áp dụng, được giữ trong backup thay vì chạy như bằng chứng cho cơ chế đã bỏ. Case recovery đổi có chủ đích: xác nhận ANGLE trước khi history hoàn tất không được dùng làm quyền mở PI về sau; test yêu cầu xác nhận mới sau history. Profile là kích thích cảm biến động học, **không phải mô hình motor/thrust vòng kín hoặc chứng minh tuning bay**.

Đã kiểm checksum: **11/11 file original** khớp manifest (kể cả tài liệu Sonnet), danh sách file không đổi; **22/22 file backup** khớp checksum. Baseline trước khi sửa đã chạy thực tế 67/67 host, 7/7 macro, 10/10 reference hash; chỉ dùng làm bằng chứng bản cũ/backup.

Backup: [manualacc_after_change_20261016/manifest.json](../../../backups/manualacc_after_change_20261016/manifest.json), gồm firmware/tài liệu cũ và bộ test cũ. Đây là bản khôi phục, không phải thư mục firmware mới được bàn giao. Bộ test hiện tại: [runner](../../../tests/manualacc/run_tests.py), [cases](../../../tests/manualacc/host_tests.cpp), [reference hashes bất biến](../../../tests/manualacc/baseline_sha256.json).

Những giới hạn còn lại:

- **Sau đặt xuống: ARM OFF trước lần thử tiếp theo; không cần reset ESP32.** Nếu vẫn armed ở FLIGHT thì I bay có thể còn. Chưa thể xác nhận sự cố thực chỉ do windup.
- Readiness/evidence có thể nhầm do áp suất/luồng cánh/nhấc bằng tay; finite/freshness/history không phát hiện hết bias hoặc số đo sai. Không có guard jump/innovation/mismatch của bản cũ.
- KF dọc vẫn correct500Hz với mẫu baro nguồn50Hz, không phải500 phép đo độc lập. DT cố định2ms chưa được đo deadline thực tế.
- Slew chỉ giới hạn collective, không mọi motor; thang nội bộ không phải lực đẩy. Anti-windup không phát hiện hỏng motor/pin/cánh hoặc thân bị giữ.
- DEGRADED không bảo đảm hạ/giữ cao; hard IMU/output fault ngắt motor không phải hạ an toàn. Không tự xác nhận touchdown hoặc tắt motor trên đất.
- Không thấy arduino-cli/arduino-builder/pio trên PATH: **chưa build target ESP32**, chưa upload, đo timing RMT/500Hz, thử motor hoặc thử bay. Host PASS không chứng minh dấu trục/motor, lực đẩy hoặc ổn định/an toàn bay.

[README vận hành](README.md) và [giải thích cảm biến→PID→mixer](GIAI_THICH_LUONG_CAM_BIEN_PID_MIXER.md) mô tả bản dựng này; [changelog](changelog_DroneFC_CRSF_DShot_ManualAcc.md) tách cơ chế hiện tại khỏi lịch sử.
