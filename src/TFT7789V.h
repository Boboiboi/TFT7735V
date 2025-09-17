#ifndef TFT7789V_H
#define TFT7789V_H

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include "font8x8.h"

// ST7735V Commands
#define ST7735_NOP         0x00
#define ST7735_SWRESET     0x01
#define ST7735_RDDID       0x04
#define ST7735_RDDST       0x09
#define ST7735_SLPIN       0x10
#define ST7735_SLPOUT      0x11
#define ST7735_PTLON       0x12
#define ST7735_NORON       0x13
#define ST7735_INVOFF      0x20
#define ST7735_INVON       0x21
#define ST7735_DISPOFF     0x28
#define ST7735_DISPON      0x29
#define ST7735_CASET       0x2A
#define ST7735_RASET       0x2B
#define ST7735_RAMWR       0x2C
#define ST7735_RAMRD       0x2E
#define ST7735_PTLAR       0x30
#define ST7735_COLMOD      0x3A
#define ST7735_MADCTL      0x36

// Display dimensions (ST7735V 1.8" 128x160)
#define ST7735_WIDTH       128
#define ST7735_HEIGHT      160

// Triple buffering configuration
#define SRAM_BUFFER_SIZE   8192    // 8KB SRAM buffer size
#define CHUNK_HEIGHT       (SRAM_BUFFER_SIZE / (ST7735_WIDTH * 2))  // Height of each chunk
#define MAX_CHUNKS         ((ST7735_HEIGHT + CHUNK_HEIGHT - 1) / CHUNK_HEIGHT)  // Total chunks needed

// Triple buffer states
typedef enum {
    BUFFER_STATE_RENDERING,    // Currently being drawn to
    BUFFER_STATE_TRANSFERRING, // Currently being transferred to display
    BUFFER_STATE_IDLE          // Available for next render
} buffer_state_t;

// Dirty rectangle structure
typedef struct {
    uint16_t x, y, w, h;
    bool valid;
} dirty_rect_t;

// Display task message structure
typedef struct {
    uint8_t chunk_idx;
    bool is_last_chunk;
    uint8_t source_buffer_idx; // Which PSRAM buffer to read from
    bool use_dirty_rect;       // Whether to use dirty rect optimization
    dirty_rect_t dirty_rect;   // Dirty rectangle region
} display_message_t;

// Color definitions
#define ST7735_BLACK       0x0000
#define ST7735_WHITE       0xFFFF
#define ST7735_RED         0xF800
#define ST7735_GREEN       0x07E0
#define ST7735_BLUE        0x001F
#define ST7735_YELLOW      0xFFE0
#define ST7735_MAGENTA     0xF81F
#define ST7735_CYAN        0x07FF

class TFT7789V {
private:
    spi_device_handle_t spi_device;
    gpio_num_t cs_pin;
    gpio_num_t dc_pin;
    gpio_num_t reset_pin;
    gpio_num_t bl_pin;
    
    bool initialized;
    bool spi_initialized;
    bool pwm_initialized;
    uint32_t spi_frequency;    uint8_t brightness_level;
      // Triple buffer framebuffer support
    uint16_t* framebuffer_a;        // PSRAM buffer A - for rendering
    uint16_t* framebuffer_b;        // PSRAM buffer B - for transferring
    uint16_t* framebuffer_c;        // PSRAM buffer C - idle/next
    uint16_t* current_framebuffer;  // Currently active framebuffer for drawing
    buffer_state_t buffer_states[3]; // State of each buffer
    uint8_t render_buffer_idx;      // Index of buffer currently being rendered to
    uint8_t transfer_buffer_idx;    // Index of buffer currently being transferred
    bool framebuffer_enabled;
    size_t framebuffer_size;
      // Double SRAM buffering support
    uint16_t* sram_buffer_a;        // First SRAM buffer (8KB)
    uint16_t* sram_buffer_b;        // Second SRAM buffer (8KB)
    uint16_t* current_sram_buffer;  // Current active SRAM buffer
    TaskHandle_t display_task_handle;
    QueueHandle_t display_queue;
    SemaphoreHandle_t display_done_semaphore;
    volatile bool display_in_progress;    volatile bool display_done_flag;
    uint8_t current_chunk;
    uint8_t total_chunks;
    
    // Dirty rectangle optimization
    dirty_rect_t dirty_rect;       // Current dirty rectangle
    bool dirty_rect_enabled;       // Whether dirty rect optimization is enabled
    bool force_full_redraw;        // Force full frame redraw flag
    
    // Private methods for SPI communication
    void spi_pre_transfer_callback(spi_transaction_t *t);
    void write_command(uint8_t cmd);
    void write_data(uint8_t data);
    void write_data16(uint16_t data);
    void write_data_buffer(const uint8_t* data, size_t len);
    void hardware_reset();
    void init_sequence();
    void init_pwm();
    void update_spi_speed();
    void apply_brightness();    // Framebuffer methods
    bool init_framebuffer();
    void free_framebuffer();
    void fb_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
    void fb_fill_screen(uint16_t color);
    void fb_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fb_draw_fast_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
    void fb_draw_fast_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
    void fb_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
    void fb_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fb_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    void fb_fill_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    void fb_draw_bitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg, bool has_bg);
    void fb_draw_rgb_bitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, const uint8_t *mask, uint16_t w, uint16_t h, bool has_mask);
    
    // Text rendering framebuffer methods
    void fb_draw_char(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size, bool has_bg);
    
    // Double buffering methods
    bool init_double_buffering();
    void free_double_buffering();
    static void display_task(void* pvParameters);
    void copy_chunk_and_send(uint8_t chunk_idx, uint8_t source_buffer_idx);
    void copy_dirty_chunk_and_send(uint8_t chunk_idx, uint8_t source_buffer_idx, const dirty_rect_t& dirty_rect);
    void send_chunk_to_display(uint16_t* buffer, uint16_t chunk_idx, uint16_t chunk_height);
    void send_dirty_chunk_to_display(uint16_t* buffer, uint16_t chunk_idx, uint16_t chunk_height, const dirty_rect_t& dirty_rect);
    
    // Dirty rectangle methods
    void expand_dirty_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    uint8_t calculate_dirty_chunks(const dirty_rect_t& dirty_rect, uint8_t& start_chunk, uint8_t& end_chunk);

public:
    // Constructor with configurable pins
    TFT7789V(gpio_num_t mosi = GPIO_NUM_11, 
             gpio_num_t sclk = GPIO_NUM_12, 
             gpio_num_t cs = GPIO_NUM_10, 
             gpio_num_t dc = GPIO_NUM_9, 
             gpio_num_t reset = GPIO_NUM_8, 
             gpio_num_t bl = GPIO_NUM_7);
    
    // Destructor
    ~TFT7789V();
      // Initialization
    bool begin(uint32_t freq_hz = 40000000);
    void end();    // Framebuffer control (enabled by default)
    bool enableFramebuffer();
    void disableFramebuffer();
    bool isFramebufferEnabled() const;    void display();  // Push framebuffer to display (async)
    void swapBuffers(); // Manual buffer swap for triple buffering
    bool displayDone() const;  // Check if async display is complete
    void waitForDisplayDone(); // Wait for async display to complete
    
    // Basic display control
    void display_on();
    void display_off();
    void set_backlight(bool state);
    void set_rotation(uint8_t rotation);
    void invert_display(bool invert);
    
    // Advanced configuration functions
    void setSPISpeed(uint32_t hz);
    void setBrightness(uint8_t level);  // 0-255, supports PWM
    void setRotation(uint8_t r);        // 0/1/2/3 for 0°/90°/180°/270°
    void setOffsets(int16_t x, int16_t y);
    void getOffsets(int16_t &x, int16_t &y) const;
    
    // Configuration getters
    uint32_t getSPISpeed() const;
    uint8_t getBrightness() const;
    uint8_t getRotation() const;
    uint16_t getWidth() const;
    uint16_t getHeight() const;    // Drawing functions (work with both framebuffer and direct mode)
    void fill_screen(uint16_t color);
    void draw_pixel(uint16_t x, uint16_t y, uint16_t color);
    void draw_fast_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
    void draw_fast_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
    void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    
    // Extended drawing functions (Adafruit/LovyanGFX compatible)
    void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
    void drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
    void drawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
    void drawBitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color);
    void drawBitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg);
    void drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, uint16_t w, uint16_t h);
    void drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, const uint8_t *mask, uint16_t w, uint16_t h);
    
    // Text rendering functions
    void setCursor(uint16_t x, uint16_t y);
    void setTextColor(uint16_t color);
    void setTextColor(uint16_t color, uint16_t bg);
    void setTextSize(uint8_t size);
    void setTextWrap(bool wrap);
    
    // Print functions (compatible with Print class)
    size_t write(uint8_t c);
    size_t print(const char* str);
    size_t print(int num);
    size_t print(float num, int decimals = 2);
    size_t println(const char* str = "");
    size_t println(int num);
    size_t println(float num, int decimals = 2);
    
    // Advanced text functions
    void drawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
    void drawText(uint16_t x, uint16_t y, const char* text, uint16_t color);
    void drawText(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg);
    void drawText(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg, uint8_t size);
    
    // Text measurement functions
    uint16_t getTextWidth(const char* text, uint8_t size = 1);
    uint16_t getTextHeight(uint8_t size = 1);
    
    // Direct SPI operations (bypass framebuffer)
    void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void push_colors(const uint16_t* colors, uint32_t len);
    void push_color(uint16_t color, uint32_t len);
      // Utility functions
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
      // Dirty rectangle optimization API (enabled by default)
    void enableDirtyRect(bool enable = true);
    void clearDirty();
    void forceFullRedraw();
    bool isDirtyRectEnabled() const;
    
    // Pin configuration structure
    struct PinConfig {
        gpio_num_t mosi;
        gpio_num_t sclk;
        gpio_num_t cs;
        gpio_num_t dc;
        gpio_num_t reset;
        gpio_num_t bl;
    };
    
private:
    PinConfig pins;
    uint16_t width, height;
    uint8_t rotation;
    int16_t x_offset;
    int16_t y_offset;
    
    // Text cursor and settings
    uint16_t cursor_x, cursor_y;
    uint16_t text_color, text_bg_color;
    uint8_t text_size;
    bool text_wrap;
    bool text_has_bg;
};

#endif // TFT7789V_H
