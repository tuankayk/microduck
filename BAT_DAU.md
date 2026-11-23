# Canard D1 · robot hai chân

Canard D1 hiện mặc lớp vỏ **Microduck** từ tệp `microduck.glb` trong Downloads. Mỏ màu cam, vỏ đầu màu ngà, cụm mắt, chân và các chi tiết cơ khí lấy từ mô hình gốc. Các cụm được căn theo khớp cổ, hông, gối và cổ chân của Canard D1. Mỏ chuyển động cùng đầu. Robot dùng dáng đi đã học và các lệnh giọng nói hiện có.

**Ban_thiet_ke.png** cho thấy ba góc nhìn. **Canard_D1.xml** là mô hình MJCF có thể chỉnh sửa và có tư thế lưu tên `dung`. Khi chuyển XML sang máy khác, chép cả thư mục **microduck** bên cạnh XML. **Di_tren_bon_be_mat.mp4** là video quay trực tiếp ngoại hình mới trên bốn mặt đường.

Ngoại hình gốc do **silva.williamsrodrigo** tạo, giấy phép CC BY 4.0. Chi tiết nguồn và các chỉnh sửa nằm ở [ghi công Microduck](microduck/NGUON.md).

Dự án chạy từ thư mục riêng. Trên máy này, trình khởi động ưu tiên Python trong `.venv` của Canard D1, sau đó tìm Python có sẵn tại `%USERPROFILE%\ROBOT\.venv`. Mã điều khiển, mẫu giọng, nhật ký và trọng số dáng đi đều thuộc Canard D1. Các thư viện cần dùng là Python, NumPy và MuJoCo 4.0.1 trên Windows. Huấn luyện hiện chạy trên CPU.

## 1. Xem con robot bước đi

Trong PowerShell mở thư mục và chạy hai dòng sau.

```powershell
cd C:\workspace\Canard_D1
.\Vit.cmd chay --san phang
```

Chọn cửa sổ MuJoCo. Nhấn **↑** để đi, **F** để tăng tốc, **Space** để dừng. **↓** đi lùi. **←/→** quay đầu nhìn. **G** gật đầu. **R** đặt robot về vị trí ban đầu. Khi đổi tốc độ, lệnh được tăng giảm từ từ để tránh giật thân. Thử nhấn F rồi Space. Robot sẽ giảm nhịp chuyển động và đứng lại trên hai bàn chân.

Đóng cửa sổ rồi đổi `phang` thành `tron`, `doc` hoặc `nhap_nho`. Sàn trơn thay hệ số ma sát. Dốc tăng độ cao theo hướng tiến, khoảng 1,4–3,0°. Nền gợn là lưới địa hình thật, với độ cao từng điểm thay đổi cỡ milimét. Để lặp lại đúng một mặt đường, thêm `--seed 1234`. Nếu dùng bản ZIP ở nơi khác, hãy chuyển PowerShell tới thư mục chứa **Vit.cmd**.

## 2. Dạy robot nghe giọng của bạn

```powershell
.\Vit.cmd giong
```

Dùng micro mặc định của Windows. Trong phòng yên, nhấn Enter rồi nói ngay cụm từ trên màn hình. Mỗi lệnh cần **ba mẫu**, mỗi mẫu có hai giây để nói. Giữ cùng khoảng cách tới micro như khi điều khiển. Sau khi thu đủ 21 mẫu, chương trình lưu đặc trưng âm thanh trong **giong_cua_toi.npz**. Tệp WAV tạm được xóa khi kết thúc, quá trình chạy ngoại tuyến. Nếu thu lại toàn bộ, mẫu mới thay mẫu cũ sau khi thu đủ.

Chạy `.\Vit.cmd nghe` rồi nói **“dừng lại”**. Dòng chữ trả về giúp kiểm tra micro và bộ nhận lệnh trước khi lái. Nếu Windows báo lỗi thiết bị, kiểm tra micro mặc định và quyền cho ứng dụng máy tính trong **Settings → Privacy & security → Microphone**. Có thể thử bằng bản ghi ngắn `.\Vit.cmd nghe --wav .\mau.wav` (WAV PCM 16-bit).

## 3. Ra lệnh trong lúc robot di chuyển

```powershell
.\Vit.cmd chay --san doc
```

Nhấn **V** trong cửa sổ MuJoCo. Khi PowerShell hiện **ĐANG NGHE**, nói một trong bảy lệnh. Nhấn V cho lần nói tiếp theo. Nếu vừa tạo lại mẫu giọng, mở lại cửa sổ mô phỏng để nạp mẫu mới.

| Nói | Robot làm gì |
|---|---|
| đi tới | Bắt đầu đi với bước ngắn |
| đi nhanh | Mở rộng bước đến mức dáng đi đã học |
| đi lùi | Đảo hướng bước |
| dừng lại | Giảm tốc rồi đứng |
| nhìn trái / nhìn phải | Xoay cổ, chân tiếp tục lệnh trước đó |
| gật đầu | Cổ thực hiện một động tác gật |

Thử chuỗi **đi tới → nhìn trái → đi nhanh → dừng lại**. Nghe chưa rõ, chương trình báo lại và giữ hành động trước đó. Phím Space dừng chủ động kể cả trong lúc đang nhận lệnh.

**Phạm vi nhận giọng:** bộ so khớp học từ cách bạn phát âm bảy cụm trên. Ở `voice.py`, mỗi mẫu được đổi thành chuỗi đặc trưng phổ MFCC. Thuật toán DTW so hai chuỗi dù tốc độ nói khác nhau. Khi hai lệnh có điểm gần nhau, chương trình yêu cầu nói lại. Để đo độ chính xác cho giọng bạn, hãy đọc từng lệnh bằng các lần nói mới và so dòng **Đã nghe** với điều bạn vừa nói. Bộ kiểm tra kỹ thuật đã đưa âm thanh tổng hợp qua đủ bảy nhãn và bộ điều khiển, đồng thời kiểm tra tiếng ồn và im lặng. Chất lượng giọng tiếng Việt thực qua micro cần thử với người nói sau khi thu mẫu.

## 4. Huấn luyện thêm một vòng

```powershell
.\Vit.cmd hoc --thu 32
```

Mỗi ứng viên mang một bộ tham số bước đi. Chương trình thay nhịp, độ dài bước, độ nhấc chân, độ lắc ngang, phần thời gian bàn chân tựa đất và cách chuyển chân. Nó chạy ứng viên trên cả bốn loại sàn, rồi chấm các ứng viên có triển vọng trên biến thể mặt đường riêng. Chương trình ưu tiên tốc độ cao hơn ít nhất 2% với dao động tốc độ bằng hoặc thấp hơn, đồng thời kiểm tra ngã, góc nghiêng, trôi ngang và khả năng dừng. Chấp nhận mới ghi `dang_dung.json`. Nếu ứng viên kém thì giữ dáng đi cũ. Kết quả từng lượt ở **lich_su**. Mở lại mô phỏng để dùng dáng đi vừa được duyệt.

Các phương trình đặt bàn chân và giữ thăng bằng ở **walker.py** do người lập trình định nghĩa. Phần tự học là tìm tham số tốt hơn bằng thử nghiệm vật lý. Thân có sáu bậc tự do, động cơ chỉ tác động lên khớp, robot di chuyển nhờ bàn chân tì mặt đất. Lượt được đánh dấu ngã nếu thân nghiêng quá 48° hoặc hình va chạm thân/đầu chạm đất.

## 5. Thi lại và đọc kết quả

```powershell
.\Vit.cmd thi
```

Cùng 16 tình huống được dùng cho dáng đi ban đầu và dáng đi đang dùng. Mỗi lượt gồm ổn định ban đầu, tiến 10 giây, sau đó ra lệnh dừng và quan sát thêm 0,8 giây. `before` và `after` xuất hiện trong PowerShell và **ket_qua_thi.json**. Tốc độ tính theo mét tiến được chia cho 10 giây. Chỉ số `ripple` là độ lệch chuẩn của vận tốc tiến sau 0,7 giây khởi động, đơn vị m/s. Số nhỏ hơn cho thấy thân tiến đều hơn.

| Chỉ số trong bài thi 16 lượt | Ban đầu | Sau học |
|---|---:|---:|
| Tốc độ tiến trung bình | 0,1247 m/s | 0,1463 m/s |
| Dao động tốc độ trung bình | 0,0474 m/s | 0,0440 m/s |
| Lượt ngã | 0/16 | 0/16 |
| Góc nghiêng lớn nhất | 4,39° | 4,71° |
| Độ lệch ngang lớn nhất ở cuối chặng tiến | 10,18 cm | 4,45 cm |

Tám lượt bổ sung để kiểm tra chuỗi hành động (hai lượt mỗi loại sàn) được lưu trong **thu_lenh.json**. Chúng kiểm tra hành động sau khi giải mã lệnh, gồm tiến, nhanh hơn, lùi, dừng và cử động đầu. Độ ổn định đã đo áp dụng cho các dốc và nền thử nêu trên. Cầu thang, sỏi rời, dốc lớn và va đẩy chưa được kiểm tra.

Nguồn hình dáng tham khảo: [NVIDIA GTC 2025 Highlights](https://images.nvidia.com/nvimages/gtc/pdf/GTC2025_Highlights_v2.pdf) và [Disney Research – Design and Control of a Bipedal Robotic Character](https://la.disneyresearch.com/publication/design-and-control-of-a-bipedal-robotic-character/).

