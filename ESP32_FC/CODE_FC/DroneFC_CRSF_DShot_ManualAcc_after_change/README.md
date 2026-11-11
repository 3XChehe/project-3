# DroneFC_CRSF_DShot_ManualAcc_after_change

Firmware thử nghiệm ESP32 WROOM32, ICM-20602/SPI, BMP388/I2C, RP3 ELRS/CRSF và GOKU G55M DShot600/RMT. Cập nhật **2026-10-16**: sửa trực tiếp thư mục này, lấy cấu hình và luồng ManualAcc gốc làm chuẩn; ManualAcc gốc chỉ đọc, giữ nguyên.

**Chưa được xác nhận đủ an toàn để bay.** Host tests không kiểm chứng lực đẩy, dấu trục, dây/chiều motor, nhiễu thực hoặc deadline 500Hz. Tháo toàn bộ cánh trước kiểm tra bàn. Test PASS không phải giấy phép thử bay.

## Bắt đầu từ đâu

Tài liệu này tập hợp thông tin cần thiết để tiếp tục làm việc với firmware trong thư mục này. Đọc các cảnh báo trước, sau đó thực hiện theo thứ tự; không cần mở các báo cáo lịch sử để làm theo các bước dưới đây.

1. **Chuẩn bị an toàn:** tháo toàn bộ cánh, ngắt nguồn ESC nếu có thể; kiểm tra dây và mức logic trước khi cấp nguồn.
2. **Chuẩn bị phần mềm:** mở sketch, cài đúng ESP32 core và thư viện theo mục cấu hình bên dưới; build trước khi nạp.
3. **Kiểm tra gia tốc kế:** nếu cần đo lại offset, làm theo mục hiệu chỉnh accel bên dưới; sau đó nạp lại firmware thường.
4. **Kiểm tra MTF-01:** cấu hình MicoLink, đấu UART, quan sát `DEBUG_MTF`; nếu cần đo bias thì làm theo mục hiệu chuẩn MTF và nạp lại firmware thường.
5. **Kiểm tra điều khiển trên bàn:** vẫn tháo cánh; kiểm tra kênh RC, ARM, chiều trục, thứ tự/chiều motor và hành vi mất tín hiệu theo các mục bên dưới. Chỉ cấp nguồn ESC sau khi kiểm tra dây và khu vực an toàn.
6. **Dừng ở kiểm tra bàn:** chưa có đủ bằng chứng để khẳng định firmware an toàn bay. Hoàn thành checklist không phải chấp thuận thử bay; cần xác minh riêng các giới hạn và chính sách failsafe.

## Sketch và cấu hình gốc

- Mở [DroneFC_CRSF_DShot_ManualAcc_after_change.ino](DroneFC_CRSF_DShot_ManualAcc_after_change.ino).
- Mục tiêu Arduino ESP32 core **2.0.17**, RMT legacy, thư viện **BasicLinearAlgebra**, Serial 500000baud; không mặc định tương thích core 3.x.
- Trong Arduino IDE, cài ESP32 core 2.0.17 và BasicLinearAlgebra, chọn đúng board ESP32 WROOM-32 và cổng kết nối thực tế, sau đó Verify/Compile trước khi Upload. Mở Serial Monitor ở 500000 baud. Nếu build lỗi, lưu nguyên log và xử lý lỗi trước khi nạp; chưa có kết quả build target được xác nhận cho thay đổi hiện tại.
- Ánh xạ RC: CH1 roll, CH2 pitch, CH3 ga, CH4 yaw, CH5 ARM, CH6 MODE; CRSF 420000 baud. Kiểm tra giá trị và chiều từng kênh trên `DEBUG_RC` trước khi cấp nguồn ESC.
- Chỉ có **ALT_HOLD** là chế độ bay được chọn; công tắc MODE dùng để xác nhận tiếp quản sau khi RC phục hồi. FAILSAFE chỉ tự động khi mất RC.
- Giữ gain gốc: rate roll/pitch 0.5/1.4/0.03; yaw 1/10/0; góc 10/0/0; climb **3/15/0**. Không kế thừa Ki climb 4 của after_change cũ. Giữ gain không có nghĩa đã tune phù hợp phần cứng.
- Giữ chân, dấu mixer, offset accel, DLPF, DT_CTRL=0.002s, KF dọc hai trạng thái/Q/R/predict-correct 500Hz. Kalman góc trở về **R_acc cố định 9**, không có dynamic accel trust.
- Cấu hình hiện tại bật DEBUG_MTF; nếu không chọn macro debug nào thì DEBUG_ACC_OFFSET là mặc định. CALIBRATE_ACCEL là công cụ đo offset riêng, dừng trước khi tạo task bay.

## ARM và ALT_HOLD trực tiếp

ARM cần frame RC thực còn mới, cạnh OFF→ON, ga<1150 và readiness baro tĩnh. Khi bị từ chối ARM, cần OFF→ON lại sau khi cảm biến sẵn sàng. Boot/reconnect với ARM đang ON không tự arm.

**Đặt drone xuống sau một lần thử: ARM OFF trước lần thử tiếp theo; không cần reset ESP32.** Disarm xóa PID/I/lịch sử D/output, setpoint lọc, phase/evidence/ramp; gửi 800→DShot0 rồi return trước PID. Firmware không có cảm biến xác nhận tiếp đất. Nếu vẫn armed sau khi đặt xuống, I bay có thể còn.

| Pha | Hành vi |
|---|---|
| WAIT_TAKEOFF=0 | Arm trực tiếp ALT: bốn motor 900, không chạy PID/I. **Motor vẫn có thể quay**, không phải disarm. |
| TAKEOFF=1 | Ga>1580 liên tục 200ms và readiness đạt mới bắt đầu ramp. Giữa cần giữ collective, lên tăng, xuống giảm; attitude PD hoạt động, mọi I khóa 0. Không nhấc lên thì không tự bật PI theo timeout. |
| FLIGHT=2 | Baro tăng>30cm so với mốc takeoff, trend dương được xác nhận thêm 300ms trên mẫu nguồn mới: seed PI rồi mở I. Đây là heuristic, không phải xác nhận airborne chắc chắn. |

Readiness: lịch sử 1s qua cửa sổ 500ms; roll/pitch<10°, mỗi gyro rate<5°/s, `|mean(acc_earth_z)|<0.01g`, baro peak-to-peak<20cm. Baro chỉ đếm timestamp mẫu mới, không đếm snapshot lặp 500Hz. Ngưỡng chưa được hiệu chuẩn theo nhiễu/luồng cánh thực.

Bằng chứng cất cánh chỉ được xét trong pha TAKEOFF theo baro; MTF-01 không tham gia vào bằng chứng này hoặc điều khiển ga.

Sau FLIGHT, ga thấp/vz≈0 không xóa I, không quay WAIT hoặc tự disarm. Climb là điều khiển **vận tốc dọc**, không có vòng giữ vị trí độ cao hoặc vị trí ngang. Cần giữa 1500, deadband 1460–1540 đặt mục tiêu 0cm/s; ngoài vùng này ánh xạ liên tục tới ±100cm/s, có lọc.

## Anti-windup và chuyển PI

- Collective trong các chế độ đang hoạt động dùng RAMP/CLIMB_PI, chặn 900–1600 và slew **+80/−150 đơn vị nội bộ/s**, tương ứng +0.16/−0.30 mỗi tick 2ms. Không còn chế độ ANGLE ga trực tiếp.
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
| DEGRADED trước FLIGHT | Chỉ giữ/giảm collective khi pilot đã tiếp quản, không tăng; riêng WAIT giữ 900/không torque. Muốn thử cất cánh lại phải disarm. |
| Mất cả RC và dữ liệu dọc | Giữ collective/cân bằng nếu IMU còn dùng được; không bảo đảm hạ, giữ cao hay tắt motor. |
| IMU/control output không hữu hạn | Latch control_fault và disarm ngay, không qua slew; không phải hạ an toàn. |

RC/baro có tuổi nguồn tối đa 200ms; hụt mutex hoặc đọc cache không làm mới timestamp. Sau lỗi dọc, hệ thống giữ DEGRADED đến khi disarm; không tự mở lại PI khi baro hồi. Chỉ disarm sau khi đã đưa máy về trạng thái an toàn; disarm trên không có thể gây rơi.

RC vừa hồi phải đổi nấc MODE khác nấc ghi nhận lúc hồi để tiếp quản. **Trước xác nhận, cần ga cao hay thấp đều không điều chỉnh collective**; sau xác nhận mới điều chỉnh tương đối trong DEGRADED. ARM OFF luôn ưu tiên với frame hợp lệ. Không tự disarm theo ga thấp/vz≈0 sau 2s; motor có thể tiếp tục quay trên đất.

## Phạm vi cảm biến và những cơ chế không mang sang

Bản này **không có** quarantine baro jump/innovation, kiểm mismatch vận tốc KF–trend, dynamic accel trust hoặc LED trạng thái runtime của after_change cũ. Kiểm finite/freshness/history không chứng minh số đo hữu hạn là đúng; bias, áp suất do luồng cánh hoặc cảm biến đóng băng vẫn có thể lọt và làm PI đổi ga sai.

Giữ kiểm ID ICM-20602=0x12/BMP388=0x50, chặn mẫu lỗi/áp suất≤0, yêu cầu đủ 50 mẫu tham chiếu baro; phân biệt no-new và fault. Baro init lỗi không tự runtime reinit. Gyro calibration startup 500×2ms mỗi đợt; không tự học bias lúc disarmed, chưa bù drift nhiệt online. Phải giữ đứng yên khi calibration.

Thang nội bộ: 800→DShot0; armed 900→48, 1600→2047. Không phải µs/RPM/Newton/%thrust; chưa xác nhận dải đó đủ lực đẩy.

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
| MTF-01 RX/TX ESP32 | 4/13 | MTF TX → GPIO4; GPIO13 dự phòng |

MTF-01 phải cấu hình xuất MicoLink, 115200 baud, 8N1; kiểm tra nguồn, mass chung, pinout và mức logic theo tài liệu module. Parser nhận message MicoLink `0x51`, kiểm tra độ dài/checksum và giải mã ToF, flow, chất lượng và status thô. Flow là giá trị raw có dấu; tài liệu giao thức mô tả cm/s tại độ cao 1 m, nhưng trục, dấu và đơn vị thực tế vẫn cần xác minh. `DEBUG_MTF` in trạng thái, khoảng cách, flow thô và bộ đếm MTF độc lập với `DEBUG_ALTITUDE`. Giá trị `range_fresh` chỉ kiểm tra tuổi khi xử lý tối đa 200 ms và dải 10–8000 mm, **không xác nhận độ tin cậy thực tế**; backlog UART có thể khiến mẫu cũ trông mới. Status chưa được diễn giải. MTF chỉ telemetry, baro vẫn là nguồn duy nhất điều khiển dọc.

### Đo bias MTF trên bàn

Tháo cánh quạt và ngắt nguồn motor/ESC nếu có thể. Đặt drone và bề mặt có họa tiết đứng yên, cảm biến hướng xuống; bật `CALIBRATE_MTF` trong file chính, nạp firmware và mở Serial Monitor 500000 baud. Chế độ này không khởi tạo ESC hoặc tạo task bay; nó không tự quay về firmware bay. Sau khi đo, tắt `CALIBRATE_MTF` và nạp lại firmware thường. Không bật đồng thời `CALIBRATE_ACCEL`.

`MTF_REFERENCE_MM` bằng 0 để chỉ đo flow; nhập khoảng cách chuẩn 10–8000 mm từ mặt tham chiếu cảm biến theo phương tia đến bề mặt để tính thêm offset ToF. Sau đếm ngược 5 giây, firmware thu tối đa 15 giây và cần ít nhất 200 frame trải trên ít nhất 5 giây. Kết quả in trung bình, độ lệch chuẩn và min/max flow, min/max ToF, chất lượng/status thô và định nghĩa gợi ý. Bias flow là giá trị trung bình raw; offset ToF = khoảng cách chuẩn − trung bình đo. Mẫu không đủ, mất dữ liệu, khoảng cách ngoài dải hoặc lỗi parser sẽ báo thất bại. Kết quả không chứng minh drone đứng yên, giá trị vật lý chính xác hay frame thực sự mới; lặp lại với nhiều khoảng cách và bề mặt, kiểm tra độ phân tán trước khi sử dụng.

`MTF_LEVER_X_M`, `MTF_LEVER_Y_M`, `MTF_LEVER_Z_M` là vector từ tâm IMU đến cảm biến, theo trục thân tiến/phải/xuống, đơn vị mét. Các tham số này cùng bias/offset mặc định bằng 0 chỉ để ghi nhận: **chưa được áp dụng** vào telemetry hoặc điều khiển. Trục, dấu và đơn vị flow vẫn cần xác minh; không suy ra chuyển động drone từ tên trục raw.

Khối tự quay từng motor lúc startup của original đã được bỏ **trong target này**; original giữ nguyên. Kiểm dấu trục/dây/chiều motor cần tháo cánh.

### Hiệu chỉnh gia tốc kế

1. Tháo cánh, ngắt nguồn ESC nếu có thể; đặt drone thật phẳng và đứng yên. Trong file chính tắt `DEBUG_MTF`, bật `CALIBRATE_ACCEL`; không bật đồng thời `CALIBRATE_MTF`.
2. Build, nạp và mở Serial Monitor 500000 baud. Sau đếm ngược 5 giây, firmware lấy 2000 mẫu cách nhau 2 ms. Chờ kết quả rồi ghi ba dòng `ACC_OFFSET_X_G`, `ACC_OFFSET_Y_G`, `ACC_OFFSET_Z_G` được in ra thay cho các giá trị tương ứng trong file chính.
3. Tắt `CALIBRATE_ACCEL`, bật `DEBUG_ACC_OFFSET`, build và nạp lại. Khi đặt phẳng, kiểm tra X≈0, Y≈0, Z≈1 g. Nếu không gần các giá trị này, kiểm tra độ phẳng và lặp lại; phép kiểm tĩnh không xác nhận rung khi motor chạy.
4. Sau kiểm tra, chọn lại đúng một macro debug mong muốn, chẳng hạn `DEBUG_MTF`, và nạp lại firmware thường. Khác với `CALIBRATE_MTF`, đường khởi động hiệu chuẩn accel có khởi tạo ESC và gửi DShot0; không dựa vào chế độ này như biện pháp cách ly điện ESC.

## Kiểm tra trên bàn và xử lý khi không đạt

Luôn tháo cánh, cố định khu vực kiểm tra và sẵn sàng ngắt nguồn. Motor có thể quay ngay sau ARM; đưa ARM OFF trước khi tiếp cận và ngắt nguồn trước khi chạm vào motor hoặc dây.

- **Không build/nạp được:** kiểm tra board, cổng, core 2.0.17 và BasicLinearAlgebra; ghi lại toàn bộ lỗi build hoặc upload. Không suy luận firmware hoạt động chỉ vì upload thành công.
- **Không có frame MTF hoặc bộ đếm lỗi tăng:** kiểm tra cấp nguồn, mass chung, mức logic, TX module→GPIO4, cấu hình MicoLink 115200 8N1; kiểm tra capture UART thực tế. `range_fresh` không xác nhận số đo đúng hoặc status hợp lệ.
- **Hiệu chuẩn MTF thất bại:** kiểm tra dữ liệu có liên tục, khoảng cách 10–8000 mm, lỗi parser và khoảng cách chuẩn; đặt yên rồi chạy lại. Khoảng gián đoạn giữa frame trên 200 ms hoặc frame cuối quá cũ cũng làm phép đo thất bại. Không tự coi giá trị bias là đã được áp dụng.
- **Không ARM được:** kiểm tra frame RC, ga dưới 1150, baro và IMU sẵn sàng, drone đứng yên; sau khi readiness đạt, chuyển ARM OFF rồi ON lại. Quan sát `DEBUG_RC` hoặc `DEBUG_ALTITUDE` và trường reason. Nếu không rõ nguyên nhân, để ARM OFF.
- **Thử RC/failsafe hoặc motor:** chỉ thao tác có chủ đích khi không có cánh; quan sát phản ứng theo bảng mất tín hiệu ở trên. Không dùng mất tín hiệu hoặc ga thấp làm cách dừng motor; chủ động ARM OFF khi an toàn và ngắt nguồn. Không thể suy ra an toàn bay từ thử nghiệm bàn.

## Phiếu đo và kiểm tra còn thiếu

Các phép dưới đây dùng để thu thập bằng chứng, **không chứng nhận máy an toàn bay**. Ghi ngày, phiên bản mã hoặc commit, thay đổi chưa commit, cấu hình debug, nguồn cấp, thiết bị đo và kết quả; đánh dấu **chưa đo** nếu không có dụng cụ. Không ghi PASS khi chưa có tiêu chí, không thay con số bằng phỏng đoán. Khi chạy motor trên bàn: tháo toàn bộ cánh, giữ người và vật cách xa motor, chuẩn bị ngắt nguồn; dừng ngay nếu motor nóng bất thường, quay sai hoặc không điều khiển được. Không chạm motor hay dây khi còn cấp nguồn.

### Thời gian vòng 500 Hz và tải xử lý

1. Tháo cánh; giữ ARM OFF và ngắt nguồn ESC nếu có thể. Trong file chính tắt `DEBUG_MTF`, bật `DEBUG_LOOP_TIME`, build và nạp firmware thường; mở Serial Monitor 500000 baud. Giữ các cảm biến/RC được kết nối như cấu hình cần đánh giá.
2. Ghi các dòng `loop_us:<số>` trong một khoảng thời gian đủ dài, ví dụ 5 phút, cùng điều kiện thử và số mẫu. Chu kỳ mục tiêu là **2000 µs**. Ghi giá trị nhỏ nhất/lớn nhất đã quan sát và số dòng khác đáng kể 2000 µs; lưu log thô để xem lại. Lặp lại khi có hoạt động RC và dữ liệu MTF, nhưng vẫn không cánh. So sánh các điều kiện, không tự đặt ngưỡng PASS chưa được thiết kế.
3. `loop_us` là khoảng thời gian giữa hai lần bắt đầu vòng control, **không phải thời gian thực thi hay phần trăm CPU**. Dòng Serial chỉ là snapshot khoảng 5 Hz, không phải toàn bộ 500 vòng/giây; có thể bỏ sót đỉnh trễ, snapshot cũ hoặc deadline miss. Giá trị gần 2000 µs trên log không chứng minh mọi vòng đúng hạn. Nếu thấy lệch lớn, mất log hoặc bất thường, ghi lại và dừng đánh giá là đạt.
4. Để đánh giá deadline và tải thực sự, cần bổ sung công cụ đo: timestamp mỗi vòng để thống kê phân phối/max/số lần trễ, đo thời gian thực thi và theo dõi tải/stack từng task trên cấu hình thực; có thể dùng GPIO và logic analyzer để kiểm chứng độc lập. Firmware hiện tại chưa cung cấp đầy đủ các chỉ số đó. Nếu chưa có công cụ hoặc tiêu chí được xác định trước, ghi **chưa xác minh**; không kết luận an toàn từ log snapshot.
5. Sau khi đo, tắt `DEBUG_LOOP_TIME`, chọn lại chế độ debug cần dùng và nạp lại. Bản có debug có thể tạo tải khác bản vận hành, vì vậy phải ghi rõ cấu hình khi so sánh.

### Dấu trục, cần điều khiển và motor

1. Tháo cánh, ngắt nguồn ESC nếu có thể. Bật `DEBUG_RC`, nạp firmware thường và xem CH1–CH6 khi di chuyển từng cần/công tắc. Ghi giá trị khi giữa và hai đầu, xác nhận CH3 là ga, CH5 ARM, CH6 MODE; đặt ga thấp và ARM OFF khi không kiểm tra. Nếu kênh không đúng hoặc chập chờn, sửa mapping/cấu hình tay điều khiển trước khi tiếp tục.
2. Chọn `DEBUG_ATTITUDE`, nạp lại. Giữ drone bằng tay **khi ESC không được cấp nguồn**, nghiêng chậm từng trục rồi trả về phẳng; ghi góc, dấu và chiều nghiêng thực tế. Đối chiếu với quy ước trục, vị trí lắp IMU và chiều điều khiển dự định; nếu đảo hoặc góc không hợp lý, dừng và kiểm tra trước khi cấp nguồn ESC. Kiểm tra tĩnh riêng không chứng minh phản hồi PID đúng dưới rung.
3. Với cánh vẫn tháo, kiểm tra riêng dây, thứ tự M1–M4 và chiều CW/CCW theo bảng chân phía trên bằng phương pháp kiểm tra motor phù hợp phần cứng. Khối tự quay từng motor đã bị bỏ, nên **không giả định firmware sẽ lần lượt chạy từng motor**. Chỉ tiến hành khi biết chính xác cách thử và cách ngắt nguồn; nếu không cô lập/nhận diện an toàn từng motor, dừng và kiểm tra bằng thiết bị phù hợp. Ghi motor thực tế, GPIO, vị trí và chiều; không sửa dấu mixer theo phỏng đoán.
4. Kiểm tra riêng phản ứng hiệu chỉnh roll/pitch/yaw theo phương pháp có thể quan sát lệnh từng motor an toàn, đối chiếu mixer trong mã. Debug hiện tại không in lệnh từng motor; pha WAIT cũng không chạy PID/torque, nên nghiêng máy trong WAIT không phải phép thử phản ứng PID. Chỉ nhìn góc cảm biến hoặc chiều quay motor không đủ chứng minh chiều phản hồi đúng. Nếu chưa có cách đo lệnh/thiết lập an toàn, ghi **chưa kiểm chứng**, không lắp cánh để thử.

### Lực đẩy

Không thể suy ra lực đẩy từ DShot, lệnh nội bộ 900–1600 hoặc RPM nếu chưa có phép đo. Đo có cánh cần **bệ đo lực đẩy chuyên dụng có che chắn, cố định phù hợp và người có kinh nghiệm**; không thử bằng cách giữ drone, đặt cân dưới drone hoặc chạy cánh trên bàn hở. Nếu chưa có bệ đo phù hợp, ghi **chưa đo** và không suy ra khả năng cất cánh hay độ an toàn.

Khi có dữ liệu đo độc lập, lưu loại motor/cánh, pin và điện áp dưới tải, thiết bị và hiệu chuẩn, điều kiện thử, lệnh, lực đo và giới hạn phương pháp. Lực tĩnh của một motor không chứng minh lực toàn máy, độ ổn định hay an toàn khi bay.

### Hành vi hệ thống và tình huống lỗi

- Khi **không cánh**, ghi log lúc khởi động, RC hợp lệ, ARM OFF→ON và disarm; đối chiếu điều kiện ARM và các pha ở trên. Để ARM OFF trước khi chạm vào máy. Không bắt buộc kích hoạt TAKEOFF trên bàn; ngưỡng và bằng chứng baro không chứng minh trạng thái thực.
- Chỉ mô phỏng mất/phục hồi RC hoặc cảm biến trong điều kiện có người giám sát, không cánh, biết cách ngắt nguồn; ghi thời điểm, phase, mode, reason, takeover và collective. So sánh với bảng tình huống ở trên. Nếu phản ứng khác dự kiến hoặc không thể dừng an toàn, ngắt nguồn và giữ nguyên log để điều tra.
- Thử trên bàn không mô phỏng được lực cánh, khí động học, áp suất do luồng cánh, rung hoặc việc tiếp đất. Chưa có quy trình/thẩm định an toàn bay ở đây; không diễn giải kết quả bàn là cho phép lắp cánh hoặc bay.

### Mẫu ghi kết quả

| Hạng mục | Cấu hình và cách đo | Kết quả/log | Đánh giá hoặc bước tiếp theo |
|---|---|---|---|
| Vòng 500 Hz / tải | | | Chưa đo / cần kiểm tra |
| RC và dấu góc IMU | | | Chưa đo / cần kiểm tra |
| Thứ tự, chiều và phản hồi motor | | | Chưa đo / cần kiểm tra |
| Lực đẩy | | | Chưa đo / cần bệ chuyên dụng |
| Mất/phục hồi RC và lỗi cảm biến | | | Chưa đo / cần kiểm tra |

## Kiểm thử, backup và giải thích

- Bộ test `Drone/tests/manualacc/run_tests.py` được nhắc trong báo cáo lịch sử hiện không có trong workspace; không áp dụng kết quả cũ cho bản MTF hiện tại.
- Báo cáo tích hợp MTF ghi nhận kiểm thử host tổng hợp trước đây: parser 17/17 và 9 nhóm máy trạng thái, với harness tạm không lưu trong repo. Không tự áp dụng kết quả đó cho các thay đổi về sau; các kiểm thử không chạy toàn bộ firmware, PID/Kalman thực, FreeRTOS hoặc phần cứng.
- Còn cần build ESP32 target, capture UART MTF, kiểm tra RC/cảm biến không cánh, dấu trục và motor, đo tải và deadline vòng 500 Hz. Cũng cần thiết kế và xác nhận chính sách mất RC/baro và kết thúc hạ cánh trước khi cân nhắc bay. Kiểm thử host với stub không thay thế các bước này.
- [Thay đổi MTF-01 và ALT_HOLD](THAY_DOI_MTF01_ALT_HOLD.md): phạm vi tích hợp, kiểm thử và giới hạn hiện tại.
- [Báo cáo bốn mục và kết quả kiểm chứng](BAO_CAO_XU_LY_MANUALACC.md) — lịch sử 2026-10-16.
- [Cảm biến→PID→mixer, phép tính và ví dụ vật lý](GIAI_THICH_LUONG_CAM_BIEN_PID_MIXER.md) — lịch sử 2026-10-16, một số chế độ không còn áp dụng.
- [Changelog và phần khác với after_change cũ](changelog_DroneFC_CRSF_DShot_ManualAcc.md) — lịch sử 2026-10-16.
- [Backup firmware/tài liệu/tests cũ và checksum](../../../backups/manualacc_after_change_20261016/manifest.json). Đây chỉ là bản khôi phục, không phải firmware bàn giao. Kết quả 67 case của bản cũ không phải kết quả bản này.

## Giấy phép và ghi công

Dựa trên firmware Nguyễn Văn Quý (Kỹ Thuật UAV), PID cascade tham khảo Carbon Aeronautics; giữ **GPL-3.0** và ghi công trong mã nguồn.
