# Thay đổi tích hợp MTF-01 và chế độ Alt Hold

## 1. Mục đích và phạm vi

Phiên bản này tích hợp cảm biến MicoAir MTF-01 để **đọc và quan sát telemetry**, đồng thời bỏ chế độ Angle có ga trực tiếp. Vòng PID ổn định góc roll/pitch và điều khiển yaw vẫn được giữ cho Alt Hold, FailSafe và Degraded.

**MTF-01 chưa tham gia điều khiển bay, giữ vị trí hoặc giữ độ cao.** BMP388 vẫn là nguồn đo duy nhất cho bộ lọc Kalman dọc, điều kiện sẵn sàng, xác nhận cất cánh và điều khiển tốc độ dọc.

> **Chưa xác nhận firmware đủ an toàn để bay.** Kết quả kiểm thử host không thay thế build ESP32, kiểm tra thiết bị, đo thời gian thực hoặc thử nghiệm hệ thống bay. Tháo toàn bộ cánh trước khi kiểm tra trên bàn.

## 2. Các thay đổi chính

### 2.1. Driver MTF-01

File: `mtf01.ino`

- Thay parser khung giả định `FE 0A` dài 12 byte bằng parser **MicoLink**.
- Parser đọc header bắt đầu bằng `EF`, kiểm tra độ dài payload, message ID và checksum tổng byte; payload tối đa 64 byte.
- Message `0x51` được giải mã với payload ít nhất 20 byte: thời gian cảm biến, khoảng cách ToF, strength, precision, trạng thái ToF, flow X/Y, quality và trạng thái flow.
- Giá trị flow X/Y là số nguyên có dấu, theo tài liệu MicoLink có đơn vị **cm/s tại độ cao 1 m**. Chưa dùng các giá trị này để điều khiển vị trí ngang.
- Có bộ đếm frame và lỗi; xử lý frame thiếu, timeout, checksum/độ dài sai và khôi phục đồng bộ sau lỗi.
- Status được xuất dưới dạng giá trị thô, **chưa được diễn giải hoặc xác minh với firmware cụ thể của module**.

Module cần được cấu hình xuất **MicoLink**. MTF-01 có nhiều chế độ giao thức; không giả định chế độ AUTO sẽ tự chọn đúng cho flight controller tự viết. Cần kiểm tra frame UART thực tế trước khi kết luận module hoạt động đúng.

### 2.2. Tách ToF khỏi bộ điều khiển dọc

File: `control_task.ino`

Khoảng cách ToF là khoảng cách theo hướng cảm biến tới bề mặt bên dưới; baro biểu diễn độ cao theo mốc áp suất tương đối. Hai giá trị không cùng hệ quy chiếu. Vì vậy firmware không còn chuyển trực tiếp giữa chúng làm đầu vào Kalman hoặc bằng chứng cất cánh.

Các khối sau sử dụng **baro**, không sử dụng MTF-01:

- Reset và cập nhật Kalman độ cao/tốc độ dọc.
- Readiness và kiểm tra trạng thái tĩnh trước ARM.
- Mốc, xu hướng và bằng chứng xác nhận cất cánh.
- Điều khiển tốc độ dọc và failsafe sử dụng điều khiển dọc.

Nếu cần đưa ToF vào điều khiển trong tương lai, phải thiết kế và kiểm chứng riêng hệ quy chiếu, ảnh hưởng độ nghiêng/bề mặt, chất lượng phép đo và chuyển nguồn an toàn. Bản hiện tại chưa thực hiện phần đó.

### 2.3. Chế độ bay và phục hồi

Files: `control_task.ino`, `DroneFC_CRSF_DShot_ManualAcc_after_change.ino`

- Bỏ lựa chọn `MODE_ANGLE` và nhánh Angle dùng ga trực tiếp; các vị trí công tắc không còn chọn chế độ này.
- Giữ vòng ổn định tư thế được gọi từ Alt Hold, FailSafe và Degraded. Bỏ Angle ga trực tiếp **không có nghĩa** bỏ khả năng tự cân bằng.
- ARM vẫn cần RC hợp lệ, cạnh ARM OFF→ON, ga thấp và readiness baro đạt yêu cầu.
- Khi dữ liệu dọc lỗi, hệ thống khóa điều khiển PI dọc và chuyển Degraded. Việc baro phục hồi **không tự mở lại Alt Hold trong phiên ARM**; trạng thái khóa được xóa khi disarm.
- Sau khi RC phục hồi, cần đổi nấc công tắc MODE khỏi vị trí được ghi nhận lúc hồi để xác nhận tiếp quản. Công tắc này không chọn lại Angle.
- Nếu mode nội bộ không hợp lệ và tới được nhánh xử lý mặc định, firmware đặt lỗi điều khiển và disarm; không rơi về ga trực tiếp.

**Alt Hold hiện là điều khiển vận tốc dọc**, không phải vòng giữ vị trí độ cao tuyệt đối. Cần ga giữa yêu cầu tốc độ dọc gần 0; firmware chưa có vòng ngoài đưa máy trở về một mốc cao cố định khi bị trôi.

### 2.4. Chia sẻ dữ liệu và debug

Files: `types.h`, `DroneFC_CRSF_DShot_ManualAcc_after_change.ino`

- `MtfData` chứa khoảng cách, strength, precision, trạng thái thô, flow, quality, thời gian cảm biến và các bộ đếm.
- Sensor task công bố dữ liệu MTF cùng timestamp xử lý frame qua mutex; nếu mutex bận, lần công bố tiếp theo vẫn lấy timestamp gắn với frame đã xử lý thay vì tự làm mới timestamp.
- Bật `DEBUG_MTF` để quan sát dòng `[MTF]`: khoảng cách, `range_fresh`, tuổi mẫu, strength, precision, status, flow, quality và bộ đếm.
- `range_fresh` chỉ cho biết khoảng cách trong khoảng 10–8000 mm và frame còn mới theo đồng hồ xử lý. Nó **không chứng minh phép đo vật lý đáng tin cậy** và không diễn giải status.
- Timestamp ESP32 được ghi khi parser xử lý, không phải khi byte thực sự tới UART. Nếu dữ liệu tồn đọng trong buffer, mẫu cũ có thể trông mới; cần kiểm tra tải, overflow và timing trên phần cứng.

`DEBUG_MTF` đang được bật trong phần cấu hình debug; nếu không chọn macro debug nào thì `DEBUG_ACC_OFFSET` là mặc định. Chỉ chọn một chế độ debug khi đo tải.

## 3. Kết nối và cấu hình MTF-01

Cấu hình hiện trong mã:

| Tín hiệu | Khai báo ESP32 | Ghi chú |
|---|---:|---|
| MTF-01 TX → ESP32 RX | GPIO 4 | UART1 RX |
| ESP32 TX → MTF-01 RX | GPIO 13 | Dự phòng; không cần cho đường đọc telemetry hiện tại |
| UART | 115200 baud, 8N1 | Cần cấu hình module dùng MicoLink |

Kiểm tra nguồn cấp, mass chung, pinout và mức logic theo tài liệu đúng phiên bản module trước khi đấu dây. GPIO/chú thích trong firmware không thay thế kiểm tra điện phần cứng.

### Hiệu chuẩn MTF trên bàn

Tháo toàn bộ cánh, ngắt nguồn ESC nếu có thể, đặt drone và bề mặt có họa tiết đứng yên, cảm biến hướng xuống. Bật `CALIBRATE_MTF`, nạp firmware và mở Serial Monitor 500000 baud. Chế độ này không khởi tạo ESC hoặc tạo task bay; không bật đồng thời `CALIBRATE_ACCEL`.

Đặt `MTF_REFERENCE_MM` bằng 0 nếu chỉ đo bias flow, hoặc nhập khoảng cách chuẩn 10–8000 mm theo phương tia để tính thêm offset ToF. Sau đếm ngược 5 giây, firmware thu tối đa 15 giây; cần ít nhất 200 frame trải trên ít nhất 5 giây. Dữ liệu bị gián đoạn, quá cũ, ngoài dải, không đủ hoặc có lỗi parser trong lúc thu sẽ báo thất bại.

Kết quả gồm trung bình, độ lệch chuẩn và min/max flow, min/max ToF, chất lượng/status thô và các định nghĩa bias/offset gợi ý. Offset ToF bằng khoảng cách chuẩn trừ trung bình đo. Các giá trị bias/offset và vector `MTF_LEVER_*_M` chỉ để ghi nhận, **chưa được áp dụng vào telemetry hay điều khiển**; kết quả cũng không xác nhận drone đứng yên, phép đo đúng hay frame thực sự mới. Lặp lại phép đo và kiểm tra trên phần cứng. Tắt `CALIBRATE_MTF` và nạp lại firmware thường sau khi đo.

## 4. Kiểm thử đã thực hiện

### Parser MicoLink

Kiểm thử host bằng `g++` với stub Arduino/HardwareSerial và đồng hồ giả: **17 kiểm tra đạt, 0 lỗi**. Các ca gồm payload little-endian, flow có dấu, checksum sai, độ dài sai, frame thiếu, timeout, frame liên tiếp, phục hồi đồng bộ, message khác và rollover đồng hồ.

Một trường hợp checksum sai chứa header giả khiến frame hợp lệ phía sau phải chờ đến timeout để được phục hồi. Sau khi sửa, kiểm tra lại cho thấy frame được khôi phục sau timeout. Đây là kiểm thử trên dữ liệu tổng hợp, chưa phải frame capture từ module thật.

### Máy trạng thái

Kiểm thử host với các hàm trạng thái/readiness được trích từ firmware và các thành phần liên quan được stub: **9 nhóm kiểm tra đạt**. Các ca gồm readiness baro, ARM, lỗi và phục hồi baro, mất/phục hồi RC, tiếp quản bằng MODE, mất đồng thời RC/baro và biên freshness.

Các kiểm thử này không chạy toàn bộ `control_step`, PID/Kalman thật, FreeRTOS hoặc động lực học bay. Harness được tạo trong thư mục tạm ngoài repo, không phải bộ test lưu kèm firmware.

### Chưa kiểm chứng

- Chưa build target ESP32: môi trường kiểm thử không có Arduino CLI hoặc PlatformIO được phát hiện. Sketch hướng tới Arduino ESP32 core 2.0.17 và thư viện BasicLinearAlgebra.
- Chưa nạp firmware, capture UART từ MTF-01, kiểm tra cách module đang cấu hình giao thức hoặc xác minh status trên phần cứng.
- Chưa đo deadline/jitter của vòng 500 Hz, tải sensor task, tràn UART hoặc ảnh hưởng debug lên baro.
- Chưa kiểm tra dây, dấu điều khiển, chiều motor, lực đẩy hoặc phản ứng khi vận hành thực tế.

## 5. Giới hạn an toàn còn lại

1. **Mất RC trong TAKEOFF trước khi xác nhận bay:** Degraded có thể giữ collective tại thời điểm mất tín hiệu. Không có bảo đảm máy tự hạ hay motor tự dừng.
2. **Mất cả RC và baro:** Degraded có thể giữ collective cuối; không bảo đảm giữ cao, hạ hoặc tắt motor.
3. **Failsafe sau khi xác nhận bay:** firmware yêu cầu tốc độ hạ, nhưng không xác nhận tiếp đất và không bảo đảm hạ an toàn hoặc tự disarm.
4. **Mất baro:** không tự phục hồi Alt Hold trong phiên ARM. Disarm trên không có thể gây rơi; chỉ disarm sau khi máy đã được đưa về trạng thái an toàn.
5. **Readiness và xác nhận cất cánh là heuristic:** mẫu baro còn mới và hữu hạn không chứng minh phép đo đúng; bias, nhiễu áp suất hoặc cảm biến đóng băng vẫn có thể ảnh hưởng điều khiển.
6. **ToF không phải ground truth:** khoảng cách thay đổi theo độ nghiêng, bề mặt, ánh sáng và vật cản; thông số dải đo phụ thuộc điều kiện thực tế.

Trước khi cân nhắc bay, cần thiết kế và xác nhận riêng chính sách mất RC/mất baro và kết thúc hạ cánh; kiểm tra phần cứng với **toàn bộ cánh đã tháo**. Không coi việc qua host test hoặc motor quay trên bàn là chứng nhận an toàn bay.

## 6. Tài liệu tham khảo

- MicoAir — [MTF-01 sensors](https://micoair.com/en/docs/sensors/sensors/mtf-01-sensors).
- MicoAir — [MicoLink protocol](https://micoair.com/en/docs/sensors/micolink).
- `README.md` trong cùng thư mục: cấu hình firmware và luồng vận hành hiện tại.
