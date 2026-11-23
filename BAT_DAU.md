# Microduck v2

Mô hình MuJoCo sử dụng khung khớp, tỉ lệ và lưới gốc của `microduck.glb`. Ở tư thế thiết kế, chiều cao là **26,08 cm**. Hai chân có đúng các cụm xoay hông, nghiêng hông, đùi, gối và cổ chân theo GLB. Khi chạy mô phỏng, bộ điều khiển thay đổi tư thế chân để giữ thăng bằng.

## Mở mô phỏng trên Windows

Giải nén cả thư mục rồi mở **Vit.cmd**. Chương trình tìm Python ở `.venv` của dự án hoặc trong `%USERPROFILE%\ROBOT\.venv`. Nếu dùng PowerShell, mở cửa sổ ngay trong thư mục vừa giải nén rồi chạy lệnh dưới đây.

```powershell
.\Vit.cmd chay --san phang --seed 810001
```

| Phím | Hành động |
| --- | --- |
| ↑ / ↓ | Đi tới / đi lùi |
| F | Đi tới nhanh hơn |
| Space | Giảm tốc rồi đứng lại |
| ← / → | Xoay đầu sang trái / phải |
| G | Gật đầu |
| R | Đặt lại robot |
| V | Nhận một lệnh giọng nói sau khi đã thu mẫu |

Đổi `phang` thành `tron`, `doc` hoặc `nhap_nho` để thử các mặt sàn khác. Sân dốc khoảng 1,4–3,0°. Sân nhấp nhô có chênh cao vài milimét.

## Xem thiết kế và kết quả thử

- **Ban_thiet_ke.png** hiển thị tư thế thiết kế ở ba góc nhìn.
- **Canard_D1.xml** mở trực tiếp trong MuJoCo. Giữ thư mục **microduck** cạnh XML. Tư thế `dung` là tư thế lắp ráp trong GLB.
- **Di_tren_bon_be_mat.mp4** quay mô phỏng trên bốn mặt sàn.
- **Vit.cmd thi** đo ba tốc độ lệnh trên bốn mặt sàn với bốn seed. Mỗi lượt chạy 10 giây rồi có 0,8 giây để dừng. Kết quả được ghi vào **ket_qua_thi.json**.
- **Vit.cmd hoc --thu 32** thử thông số bước đi. Chương trình kiểm tra lại trên các mặt sàn riêng, kể cả lúc đi lùi, trước khi lưu bộ tham số mới vào **dang_dung.json**. Bộ tham số Canard D1 cũ thuộc một khung kích thước khác.
- **Vit.cmd giong** thu ba mẫu cho mỗi lệnh. **Vit.cmd nghe** thử nhận lệnh từ micro. Nếu đã thu mẫu ở thư mục trước, có thể mang tệp **giong_cua_toi.npz** sang.

Bộ điều khiển dùng quỹ đạo bàn chân, động học ngược và phản hồi tư thế thân. MuJoCo tính chuyển động của thân tự do qua lực từ chân lên mặt sàn. Các thông số khối lượng, giới hạn khớp và vật liệu phản chiếu được xấp xỉ cho mô phỏng. [Ghi công mô hình gốc](microduck/NGUON.md)


