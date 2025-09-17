# TFT7735V

Thư viện điều khiển màn hình ST7735V 1.8" 128x160 (RGB565) hiệu năng cao cho ESP32 (ESP-IDF/Arduino/PlatformIO).

### Tính năng
- Triple framebuffer trong PSRAM (A/B/C) giúp vẽ mượt, tránh xé hình khi hoán đổi buffer
- Double buffering trong SRAM (2x8KB) để truyền dữ liệu theo từng “chunk” tối ưu qua SPI
- Dirty Rectangle: chỉ gửi vùng thay đổi, tăng tốc độ làm tươi khi cập nhật cục bộ
- Hiển thị bất đồng bộ bằng FreeRTOS task + queue, có `displayDone()` và `waitForDisplayDone()`
- Điều khiển đèn nền PWM (LEDC, 8-bit), hoặc bật/tắt khi PWM không có sẵn
- Văn bản với bộ font 8x8 ASCII (32–127), hỗ trợ nền, kích thước, wrap
- Vẽ cơ bản và mở rộng: pixel/line/rect/circle/bitmap 1-bit, RGB565, mask
- Tùy chỉnh tốc độ SPI, xoay màn (0/90/180/270), đảo màu, bật/tắt hiển thị
- Offset cột/hàng để căn lệch panel (thường gặp trên ST7735)

### Yêu cầu
- ESP32 + SPI, PSRAM khuyến nghị để dùng triple framebuffer
- ESP-IDF hoặc Arduino (PlatformIO framework = arduino)

### Cài đặt (PlatformIO)
- Thêm repo vào `lib_deps` hoặc sao chép thư viện vào `lib/`
- Cấu trúc đã chuẩn PlatformIO: mã nguồn trong `src/`, `library.json`

### Khởi tạo nhanh (Arduino/PlatformIO)
```cpp
#include "TFT7735V.h"

TFT7735V tft; // có thể truyền chân trong constructor nếu khác mặc định

void setup() {
  tft.begin(40000000);        // SPI 40MHz
  tft.setRotation(1);         // 0..3
  tft.setBrightness(200);     // 0..255 (PWM)
  tft.setOffsets(0, 0);       // căn lệch nếu panel dịch

  tft.fill_screen(0x0000);    // đen
  tft.drawRect(10,10,100,60, 0x07E0);  // viền xanh lục
  tft.fillRect(12,12,96,56,  0x001F);  // nền xanh dương
  tft.drawText(20,20, "Hello", 0xFFFF, 0x001F, 2);

  tft.display();
  tft.waitForDisplayDone();
}

void loop() {
  // cập nhật nhỏ: dirty-rect sẽ tối ưu băng thông
  tft.drawText(20, 90, "FPS: 60", 0xFFE0, 0x0000, 2);
  tft.display();
}
```

## API

### Khởi tạo / vòng đời
- `TFT7735V(gpio_num_t mosi=GPIO_NUM_11, gpio_num_t sclk=GPIO_NUM_12, gpio_num_t cs=GPIO_NUM_10, gpio_num_t dc=GPIO_NUM_9, gpio_num_t reset=GPIO_NUM_8, gpio_num_t bl=GPIO_NUM_7)`
- `~TFT7735V()`
- `bool begin(uint32_t freq_hz = 40000000)`
- `void end()`

### Framebuffer & hiển thị
- `bool enableFramebuffer()` / `void disableFramebuffer()` / `bool isFramebufferEnabled() const`
- `void display()` — đẩy framebuffer ra màn (bất đồng bộ)
- `bool displayDone() const` — kiểm tra đã xong chưa
- `void waitForDisplayDone()` — chờ hiển thị xong
- `void swapBuffers()` — hoán đổi buffer render thủ công

### Điều khiển cơ bản
- `void display_on()` / `void display_off()`
- `void invert_display(bool invert)`
- `void set_backlight(bool state)`
- `void setRotation(uint8_t r)` hoặc `void set_rotation(uint8_t rotation)`
- `void setSPISpeed(uint32_t hz)` / `uint32_t getSPISpeed() const`
- `void setBrightness(uint8_t level)` / `uint8_t getBrightness() const`
- `uint8_t getRotation() const`
- `uint16_t getWidth() const` / `uint16_t getHeight() const`
- Offset căn panel: `void setOffsets(int16_t x, int16_t y)`, `void getOffsets(int16_t &x, int16_t &y) const`

### Vẽ cơ bản (làm việc với cả framebuffer hoặc chế độ trực tiếp)
- `void fill_screen(uint16_t color)`
- `void draw_pixel(uint16_t x, uint16_t y, uint16_t color)`
- `void draw_fast_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color)` / alias `drawFastVLine`
- `void draw_fast_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)` / alias `drawFastHLine`
- `void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)` / alias `fillRect`
- `void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)`
- `void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)`
- `void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)` / `void fillCircle(...)`
- `uint16_t color565(uint8_t r, uint8_t g, uint8_t b)`

### Bitmap / RGB565
- Monochrome 1-bit: `void drawBitmap(x, y, const uint8_t *bitmap, w, h, color)`
- Monochrome 1-bit có nền: `void drawBitmap(x, y, const uint8_t *bitmap, w, h, color, bg)`
- RGB565: `void drawRGBBitmap(x, y, const uint16_t *bitmap, w, h)`
- RGB565 + mask: `void drawRGBBitmap(x, y, const uint16_t *bitmap, const uint8_t *mask, w, h)`

### Văn bản
- Thiết lập: `setCursor(x,y)`, `setTextColor(color)` / `setTextColor(color, bg)`, `setTextSize(size)`, `setTextWrap(bool)`
- Ghi: `size_t write(uint8_t c)`, `size_t print(...)`, `size_t println(...)`
- Vẽ trực tiếp: `drawChar(...)`, `drawText(...)`
- Kích thước chữ: `getTextWidth(text, size)`, `getTextHeight(size)`

### Dirty Rectangle (tối ưu băng thông)
- `void enableDirtyRect(bool enable=true)`
- `void clearDirty()`
- `void forceFullRedraw()`
- `bool isDirtyRectEnabled() const`

### SPI trực tiếp (bỏ qua framebuffer)
- `void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)`
- `void push_colors(const uint16_t* colors, uint32_t len)`
- `void push_color(uint16_t color, uint32_t len)`

## Ghi chú
- Nếu panel của bạn bị lệch vùng hiển thị, dùng `setOffsets(x, y)` để căn chuẩn
- `setRotation()` có thể ảnh hưởng cách panel cần offset — thử các giá trị nhỏ ± vài pixel
- Không có PSRAM vẫn dùng được, nhưng nên tắt/bớt framebuffer để tiết kiệm RAM

## Giấy phép
MIT