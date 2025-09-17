#include "TFT7735V.h"
#include <cstring>
#include <algorithm>

static const char* TAG = "TFT7735V";

// Pre-transfer callback for setting DC pin
void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t) {
    TFT7735V* tft = (TFT7735V*)t->user;
    int dc = (int)t->user;
    gpio_set_level((gpio_num_t)dc, (t->user != nullptr) ? 1 : 0);
}

TFT7735V::TFT7735V(gpio_num_t mosi, gpio_num_t sclk, gpio_num_t cs, 
                   gpio_num_t dc, gpio_num_t reset, gpio_num_t bl) {
    pins.mosi = mosi;
    pins.sclk = sclk;
    pins.cs = cs;
    pins.dc = dc;
    pins.reset = reset;
    pins.bl = bl;
    
    cs_pin = cs;
    dc_pin = dc;
    reset_pin = reset;
    bl_pin = bl;
    
    initialized = false;
    spi_initialized = false;
    pwm_initialized = false;
    spi_frequency = 40000000; // Default 40MHz
    brightness_level = 255;   // Default full brightness
    width = ST7735_WIDTH;
    height = ST7735_HEIGHT;
    rotation = 0;
    x_offset = 0;
    y_offset = 0;
    
    // Initialize text settings
    cursor_x = 0;
    cursor_y = 0;
    text_color = ST7735_WHITE;
    text_bg_color = ST7735_BLACK;
    text_size = 1;
    text_wrap = true;
    text_has_bg = false;

    spi_device = nullptr;    // Triple buffer framebuffer initialization - ALWAYS ENABLED
    framebuffer_a = nullptr;
    framebuffer_b = nullptr;
    framebuffer_c = nullptr;
    current_framebuffer = nullptr;
    framebuffer_enabled = true;  // Default enabled
    framebuffer_size = ST7735_WIDTH * ST7735_HEIGHT * sizeof(uint16_t);
    
    // Initialize buffer states and indices
    buffer_states[0] = BUFFER_STATE_IDLE;
    buffer_states[1] = BUFFER_STATE_IDLE;
    buffer_states[2] = BUFFER_STATE_IDLE;
    render_buffer_idx = 0;      // Start with buffer A
    transfer_buffer_idx = 1;    // Buffer B ready for transfer
      // Double buffering initialization
    sram_buffer_a = nullptr;
    sram_buffer_b = nullptr;
    current_sram_buffer = nullptr;
    display_task_handle = nullptr;
    display_queue = nullptr;
    display_done_semaphore = nullptr;
    display_in_progress = false;
    display_done_flag = true;
    current_chunk = 0;
    total_chunks = MAX_CHUNKS;
    dirty_rect_enabled = true;   // Default enabled
    force_full_redraw = false;
}

TFT7735V::~TFT7735V() {
    end();
    free_framebuffer();
    free_double_buffering();
}

bool TFT7735V::begin(uint32_t freq_hz) {
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }
    
    // Store SPI frequency
    if (freq_hz > 0) {
        spi_frequency = freq_hz;
    }
    
    ESP_LOGI(TAG, "Initializing TFT7735V display with SPI freq: %lu Hz", spi_frequency);
    
    // Configure GPIO pins
    gpio_config_t io_conf = {};
    
    // Configure DC pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << dc_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // Configure RESET pin
    if (reset_pin != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << reset_pin);
        gpio_config(&io_conf);
        gpio_set_level(reset_pin, 1);
    }
      // Configure Backlight pin and initialize PWM if needed
    if (bl_pin != GPIO_NUM_NC) {
        init_pwm();
        apply_brightness(); // Apply current brightness level
    }
    
    // Initialize SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = pins.mosi;
    buscfg.miso_io_num = -1; // Not used
    buscfg.sclk_io_num = pins.sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096; // Maximum transfer size in bytes
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }
    spi_initialized = true;
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = spi_frequency;
    devcfg.mode = 0; // SPI mode 0
    devcfg.spics_io_num = cs_pin;
    devcfg.queue_size = 7;
    devcfg.pre_cb = nullptr; // We'll handle DC manually
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return false;
    }
    
    // Hardware reset
    hardware_reset();
      // Initialize display
    init_sequence();
    
    // Automatically initialize framebuffer system with triple buffering and dirty rect
    ESP_LOGI(TAG, "Initializing framebuffer system (triple buffer + dirty rect)...");
    if (!init_framebuffer()) {
        ESP_LOGE(TAG, "Failed to initialize framebuffer system");
        return false;
    }
    
    if (!init_double_buffering()) {
        ESP_LOGE(TAG, "Failed to initialize double buffering system");
        free_framebuffer();
        return false;
    }
    
    // Set current framebuffer to first buffer
    current_framebuffer = framebuffer_a;
    
    ESP_LOGI(TAG, "Framebuffer system initialized successfully");
    ESP_LOGI(TAG, "- Triple buffering: ENABLED");
    ESP_LOGI(TAG, "- Dirty rectangle optimization: ENABLED");
    ESP_LOGI(TAG, "- Total PSRAM usage: %d KB", (framebuffer_size * 3) / 1024);
    ESP_LOGI(TAG, "- Total SRAM usage: %d KB", (SRAM_BUFFER_SIZE * 2) / 1024);
    
    initialized = true;
    ESP_LOGI(TAG, "TFT7735V initialized successfully with high-performance mode");
    return true;
}

void TFT7735V::end() {
    if (!initialized) return;
    
    // Clean up framebuffer system first
    free_framebuffer();
    free_double_buffering();
    
    if (spi_device) {
        spi_bus_remove_device(spi_device);
        spi_device = nullptr;
    }
    
    if (spi_initialized) {
        spi_bus_free(SPI2_HOST);
        spi_initialized = false;
    }
    
    if (pwm_initialized) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        pwm_initialized = false;
    }
    
    initialized = false;
    ESP_LOGI(TAG, "TFT7735V deinitialized");
}

void TFT7735V::hardware_reset() {
    if (reset_pin != GPIO_NUM_NC) {
        gpio_set_level(reset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(reset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

void TFT7735V::write_command(uint8_t cmd) {
    gpio_set_level(dc_pin, 0); // Command mode
    
    spi_transaction_t t = {};
    t.length = 8; // 8 bits
    t.tx_data[0] = cmd;
    t.flags = SPI_TRANS_USE_TXDATA;
    
    esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
    }
}

void TFT7735V::write_data(uint8_t data) {
    gpio_set_level(dc_pin, 1); // Data mode
    
    spi_transaction_t t = {};
    t.length = 8; // 8 bits
    t.tx_data[0] = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    
    esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(ret));
    }
}

void TFT7735V::write_data16(uint16_t data) {
    gpio_set_level(dc_pin, 1); // Data mode
    
    spi_transaction_t t = {};
    t.length = 16; // 16 bits
    t.tx_data[0] = data >> 8;
    t.tx_data[1] = data & 0xFF;
    t.flags = SPI_TRANS_USE_TXDATA;
    
    esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send 16-bit data: %s", esp_err_to_name(ret));
    }
}

void TFT7735V::write_data_buffer(const uint8_t* data, size_t len) {
    if (len == 0) return;
    
    gpio_set_level(dc_pin, 1); // Data mode
    
    spi_transaction_t t = {};
    t.length = len * 8; // Length in bits
    t.tx_buffer = data;
    
    esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send data buffer: %s", esp_err_to_name(ret));
    }
}

void TFT7735V::init_sequence() {
    ESP_LOGI(TAG, "Starting display initialization sequence");
    
    // Software reset
    write_command(ST7735_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    write_command(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Color mode - 16-bit color
    write_command(ST7735_COLMOD);
    write_data(0x05); // 16-bit color (RGB565)
    
    // Memory access control
    write_command(ST7735_MADCTL);
    write_data(0x00); // Default orientation
    
    // Column address set (0..127)
    write_command(ST7735_CASET);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0x7F); // 127
    
    // Row address set (0..159)
    write_command(ST7735_RASET);
    write_data(0x00);
    write_data(0x00);
    write_data(0x00);
    write_data(0x9F); // 159
    
    // Normal display mode on
    write_command(ST7735_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Display on
    write_command(ST7735_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Display initialization sequence completed");
}

void TFT7735V::display_on() {
    write_command(ST7735_DISPON);
}

void TFT7735V::display_off() {
    write_command(ST7735_DISPOFF);
}

void TFT7735V::set_backlight(bool state) {
    if (bl_pin != GPIO_NUM_NC) {
        gpio_set_level(bl_pin, state ? 1 : 0);
    }
}

void TFT7735V::invert_display(bool invert) {
    write_command(invert ? ST7735_INVON : ST7735_INVOFF);
}

void TFT7735V::set_rotation(uint8_t rotation) {
    this->rotation = rotation % 4;
    uint8_t madctl = 0;
    
    switch (this->rotation) {
        case 0: // Portrait (0°)
            madctl = 0x00; // No rotation
            width = ST7735_WIDTH;
            height = ST7735_HEIGHT;
            break;
        case 1: // Landscape (90° clockwise)
            madctl = 0x60; // MV=1, MX=1, MY=0, RGB=0
            width = ST7735_HEIGHT;
            height = ST7735_WIDTH;
            break;
        case 2: // Portrait inverted (180°)
            madctl = 0xC0; // MX=1, MY=1, MV=0, RGB=0
            width = ST7735_WIDTH;
            height = ST7735_HEIGHT;
            break;
        case 3: // Landscape inverted (270° clockwise)
            madctl = 0xA0; // MV=1, MX=0, MY=1, RGB=0
            width = ST7735_HEIGHT;
            height = ST7735_WIDTH;
            break;
    }
      ESP_LOGI(TAG, "Setting rotation %d, MADCTL=0x%02X, Width=%d, Height=%d", 
             this->rotation, madctl, width, height);
    write_command(ST7735_MADCTL);
    write_data(madctl);
    
    // Add small delay after MADCTL command
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Reset address window to full screen after rotation
    set_addr_window(0, 0, width - 1, height - 1);
}

void TFT7735V::set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address set
    uint16_t sx0 = x0 + (x_offset >= 0 ? (uint16_t)x_offset : 0);
    uint16_t sy0 = y0 + (y_offset >= 0 ? (uint16_t)y_offset : 0);
    uint16_t sx1 = x1 + (x_offset >= 0 ? (uint16_t)x_offset : 0);
    uint16_t sy1 = y1 + (y_offset >= 0 ? (uint16_t)y_offset : 0);

    write_command(ST7735_CASET);
    write_data(sx0 >> 8);
    write_data(sx0 & 0xFF);
    write_data(sx1 >> 8);
    write_data(sx1 & 0xFF);
    
    // Row address set
    write_command(ST7735_RASET);
    write_data(sy0 >> 8);
    write_data(sy0 & 0xFF);
    write_data(sy1 >> 8);
    write_data(sy1 & 0xFF);
    
    // Memory write
    write_command(ST7735_RAMWR);
}

void TFT7735V::setOffsets(int16_t x, int16_t y) {
    x_offset = x;
    y_offset = y;
}

void TFT7735V::getOffsets(int16_t &x, int16_t &y) const {
    x = x_offset;
    y = y_offset;
}

void TFT7735V::draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_pixel(x, y, color);
    } else {
        // Direct mode - original implementation
        if (x >= width || y >= height) return;
        
        set_addr_window(x, y, x, y);
        write_data16(color);
    }
}

void TFT7735V::fill_screen(uint16_t color) {
    if (framebuffer_enabled) {
        fb_fill_screen(color);
    } else {
        // Direct mode - original implementation
        fill_rect(0, 0, width, height, color);
    }
}

void TFT7735V::fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (framebuffer_enabled) {
        fb_fill_rect(x, y, w, h, color);
    } else {
        // Direct mode - original implementation
        if (x >= width || y >= height) return;
        if (x + w > width) w = width - x;
        if (y + h > height) h = height - y;
        
        set_addr_window(x, y, x + w - 1, y + h - 1);
        
        uint32_t total_pixels = w * h;
        push_color(color, total_pixels);
    }
}

void TFT7735V::push_color(uint16_t color, uint32_t len) {
    gpio_set_level(dc_pin, 1); // Data mode
    
    // Create buffer for efficient transfer
    const size_t chunk_size = 1024;
    uint16_t buffer[chunk_size];
    
    // Fill buffer with color
    for (size_t i = 0; i < chunk_size; i++) {
        buffer[i] = __builtin_bswap16(color); // Swap bytes for big-endian transmission
    }
    
    while (len > 0) {
        size_t current_chunk = (len > chunk_size) ? chunk_size : len;
        
        spi_transaction_t t = {};
        t.length = current_chunk * 16; // Length in bits
        t.tx_buffer = buffer;
        
        esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to push color: %s", esp_err_to_name(ret));
            break;
        }
        
        len -= current_chunk;
    }
}

void TFT7735V::push_colors(const uint16_t* colors, uint32_t len) {
    gpio_set_level(dc_pin, 1); // Data mode
    
    // Convert to big-endian and send
    const size_t chunk_size = 512;
    uint16_t buffer[chunk_size];
    
    uint32_t remaining = len;
    const uint16_t* src = colors;
    
    while (remaining > 0) {
        size_t current_chunk = (remaining > chunk_size) ? chunk_size : remaining;
        
        // Convert endianness
        for (size_t i = 0; i < current_chunk; i++) {
            buffer[i] = __builtin_bswap16(src[i]);
        }
        
        spi_transaction_t t = {};
        t.length = current_chunk * 16; // Length in bits
        t.tx_buffer = buffer;
        
        esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to push colors: %s", esp_err_to_name(ret));
            break;
        }
        
        src += current_chunk;
        remaining -= current_chunk;
    }
}

void TFT7735V::draw_fast_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_fast_vline(x, y, h, color);
    } else {
        fill_rect(x, y, 1, h, color);
    }
}

void TFT7735V::draw_fast_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_fast_hline(x, y, w, color);
    } else {
        fill_rect(x, y, w, 1, color);
    }
}

uint16_t TFT7735V::color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Text rendering functions implementation
void TFT7735V::setCursor(uint16_t x, uint16_t y) {
    cursor_x = x;
    cursor_y = y;
}

void TFT7735V::setTextColor(uint16_t color) {
    text_color = color;
    text_has_bg = false;
}

void TFT7735V::setTextColor(uint16_t color, uint16_t bg) {
    text_color = color;
    text_bg_color = bg;
    text_has_bg = true;
}

void TFT7735V::setTextSize(uint8_t size) {
    text_size = (size > 0) ? size : 1;
}

void TFT7735V::setTextWrap(bool wrap) {
    text_wrap = wrap;
}

void TFT7735V::drawChar(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {
    if (framebuffer_enabled) {
        fb_draw_char(x, y, c, color, bg, size, text_has_bg);
    } else {
        // Direct mode character drawing
        if (c < FONT8X8_FIRST_CHAR || c > FONT8X8_LAST_CHAR) {
            c = '?'; // Default character for unsupported chars
        }
        
        const uint8_t* char_data = font8x8_basic[c - FONT8X8_FIRST_CHAR];
          for (int8_t row = 0; row < FONT8X8_HEIGHT; row++) {
            uint8_t line = char_data[row];
            for (int8_t col = 0; col < FONT8X8_WIDTH; col++) {
                if (line & (0x01 << col)) {
                    // Draw foreground pixel(s)
                    for (uint8_t sy = 0; sy < size; sy++) {
                        for (uint8_t sx = 0; sx < size; sx++) {
                            draw_pixel(x + col * size + sx, y + row * size + sy, color);
                        }
                    }
                } else if (text_has_bg) {
                    // Draw background pixel(s)
                    for (uint8_t sy = 0; sy < size; sy++) {
                        for (uint8_t sx = 0; sx < size; sx++) {
                            draw_pixel(x + col * size + sx, y + row * size + sy, bg);
                        }
                    }
                }
            }
        }
    }
}

void TFT7735V::drawText(uint16_t x, uint16_t y, const char* text, uint16_t color) {
    while (*text) {
        drawChar(x, y, *text, color, text_bg_color, 1);
        x += FONT8X8_WIDTH;
        text++;
    }
}

void TFT7735V::drawText(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg) {
    while (*text) {
        drawChar(x, y, *text, color, bg, 1);
        x += FONT8X8_WIDTH;
        text++;
    }
}

void TFT7735V::drawText(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg, uint8_t size) {
    while (*text) {
        drawChar(x, y, *text, color, bg, size);
        x += FONT8X8_WIDTH * size;
        text++;
    }
}

size_t TFT7735V::write(uint8_t c) {
    if (c == '\n') {
        cursor_y += FONT8X8_HEIGHT * text_size;
        cursor_x = 0;
    } else if (c == '\r') {
        cursor_x = 0;
    } else if (c >= FONT8X8_FIRST_CHAR && c <= FONT8X8_LAST_CHAR) {
        // Check for text wrapping
        if (text_wrap && (cursor_x + FONT8X8_WIDTH * text_size) > width) {
            cursor_x = 0;
            cursor_y += FONT8X8_HEIGHT * text_size;
        }
        
        drawChar(cursor_x, cursor_y, c, text_color, text_bg_color, text_size);
        cursor_x += FONT8X8_WIDTH * text_size;
    }
    return 1;
}

size_t TFT7735V::print(const char* str) {
    size_t n = 0;
    while (*str) {
        n += write(*str++);
    }
    return n;
}

size_t TFT7735V::print(int num) {
    char buffer[12]; // Enough for 32-bit int
    sprintf(buffer, "%d", num);
    return print(buffer);
}

size_t TFT7735V::print(float num, int decimals) {
    char buffer[32];
    sprintf(buffer, "%.*f", decimals, num);
    return print(buffer);
}

size_t TFT7735V::println(const char* str) {
    size_t n = print(str);
    n += write('\n');
    return n;
}

size_t TFT7735V::println(int num) {
    size_t n = print(num);
    n += write('\n');
    return n;
}

size_t TFT7735V::println(float num, int decimals) {
    size_t n = print(num, decimals);
    n += write('\n');
    return n;
}

uint16_t TFT7735V::getTextWidth(const char* text, uint8_t size) {
    return strlen(text) * FONT8X8_WIDTH * size;
}

uint16_t TFT7735V::getTextHeight(uint8_t size) {
    return FONT8X8_HEIGHT * size;
}

// Extended drawing functions (Adafruit/LovyanGFX compatible)
void TFT7735V::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_line(x0, y0, x1, y1, color);
    } else {
        // Direct mode implementation - simple Bresenham line algorithm
        int16_t dx = abs(x1 - x0);
        int16_t dy = abs(y1 - y0);
        int16_t sx = x0 < x1 ? 1 : -1;
        int16_t sy = y0 < y1 ? 1 : -1;
        int16_t err = dx - dy;
        
        while (true) {
            draw_pixel(x0, y0, color);
            
            if (x0 == x1 && y0 == y1) break;
            
            int16_t e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

void TFT7735V::drawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    draw_fast_vline(x, y, h, color);
}

void TFT7735V::drawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    draw_fast_hline(x, y, w, color);
}

void TFT7735V::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_rect(x, y, w, h, color);
    } else {
        // Direct mode - draw rectangle outline
        if (w == 0 || h == 0) return;
        drawFastHLine(x, y, w, color);           // Top edge
        drawFastHLine(x, y + h - 1, w, color);  // Bottom edge
        if (h > 2) {
            drawFastVLine(x, y + 1, h - 2, color);         // Left edge
            drawFastVLine(x + w - 1, y + 1, h - 2, color); // Right edge
        }
    }
}

void TFT7735V::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    fill_rect(x, y, w, h, color);
}

void TFT7735V::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_circle(x0, y0, r, color);
    } else {
        // Direct mode - Bresenham circle algorithm
        int16_t x = r;
        int16_t y = 0;
        int16_t err = 0;
        
        while (x >= y) {
            draw_pixel(x0 + x, y0 + y, color);
            draw_pixel(x0 + y, y0 + x, color);
            draw_pixel(x0 - y, y0 + x, color);
            draw_pixel(x0 - x, y0 + y, color);
            draw_pixel(x0 - x, y0 - y, color);
            draw_pixel(x0 - y, y0 - x, color);
            draw_pixel(x0 + y, y0 - x, color);
            draw_pixel(x0 + x, y0 - y, color);
            
            if (err <= 0) {
                y += 1;
                err += 2 * y + 1;
            }
            if (err > 0) {
                x -= 1;
                err -= 2 * x + 1;
            }
        }
    }
}

void TFT7735V::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    if (framebuffer_enabled) {
        fb_fill_circle(x0, y0, r, color);
    } else {
        // Direct mode - filled circle using horizontal lines
        int16_t x = r;
        int16_t y = 0;
        int16_t err = 0;
        
        while (x >= y) {
            drawFastHLine(x0 - x, y0 + y, 2 * x + 1, color);
            drawFastHLine(x0 - x, y0 - y, 2 * x + 1, color);
            drawFastHLine(x0 - y, y0 + x, 2 * y + 1, color);
            drawFastHLine(x0 - y, y0 - x, 2 * y + 1, color);
            
            if (err <= 0) {
                y += 1;
                err += 2 * y + 1;
            }
            if (err > 0) {
                x -= 1;
                err -= 2 * x + 1;
            }
        }
    }
}

void TFT7735V::drawBitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color) {
    if (framebuffer_enabled) {
        fb_draw_bitmap(x, y, bitmap, w, h, color, 0, false);
    } else {
        // Direct mode - draw monochrome bitmap
        for (uint16_t row = 0; row < h; row++) {
            for (uint16_t col = 0; col < w; col++) {
                uint16_t byte_idx = (row * ((w + 7) / 8)) + (col / 8);
                uint8_t bit_idx = 7 - (col % 8);
                if (bitmap[byte_idx] & (1 << bit_idx)) {
                    draw_pixel(x + col, y + row, color);
                }
            }
        }
    }
}

void TFT7735V::drawBitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg) {
    if (framebuffer_enabled) {
        fb_draw_bitmap(x, y, bitmap, w, h, color, bg, true);
    } else {
        // Direct mode - draw monochrome bitmap with background
        for (uint16_t row = 0; row < h; row++) {
            for (uint16_t col = 0; col < w; col++) {
                uint16_t byte_idx = (row * ((w + 7) / 8)) + (col / 8);
                uint8_t bit_idx = 7 - (col % 8);
                uint16_t pixel_color = (bitmap[byte_idx] & (1 << bit_idx)) ? color : bg;
                draw_pixel(x + col, y + row, pixel_color);
            }
        }
    }
}

void TFT7735V::drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, uint16_t w, uint16_t h) {
    if (framebuffer_enabled) {
        fb_draw_rgb_bitmap(x, y, bitmap, nullptr, w, h, false);
    } else {
        // Direct mode - draw RGB565 bitmap
        for (uint16_t row = 0; row < h; row++) {
            for (uint16_t col = 0; col < w; col++) {
                uint16_t pixel_color = bitmap[row * w + col];
                draw_pixel(x + col, y + row, pixel_color);
            }
        }
    }
}

void TFT7735V::drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, const uint8_t *mask, uint16_t w, uint16_t h) {
    if (framebuffer_enabled) {
        fb_draw_rgb_bitmap(x, y, bitmap, mask, w, h, true);
    } else {
        // Direct mode - draw RGB565 bitmap with mask
        for (uint16_t row = 0; row < h; row++) {
            for (uint16_t col = 0; col < w; col++) {
                uint16_t byte_idx = (row * ((w + 7) / 8)) + (col / 8);
                uint8_t bit_idx = 7 - (col % 8);
                if (mask[byte_idx] & (1 << bit_idx)) {
                    uint16_t pixel_color = bitmap[row * w + col];
                    draw_pixel(x + col, y + row, pixel_color);
                }
            }
        }
    }
}

// Advanced configuration functions
void TFT7735V::setSPISpeed(uint32_t hz) {
    if (hz == 0) {
        ESP_LOGW(TAG, "Invalid SPI speed, ignoring");
        return;
    }
    
    spi_frequency = hz;
    ESP_LOGI(TAG, "SPI speed set to: %lu Hz", hz);
    
    // If already initialized, update the SPI device speed
    if (initialized && spi_device) {
        update_spi_speed();
    }
}

void TFT7735V::setBrightness(uint8_t level) {
    brightness_level = level;
    ESP_LOGI(TAG, "Brightness set to: %d/255", level);
    
    // Apply brightness immediately if initialized
    if (initialized || pwm_initialized) {
        apply_brightness();
    }
}

void TFT7735V::setRotation(uint8_t r) {
    set_rotation(r); // Use existing implementation
}

// Configuration getters
uint32_t TFT7735V::getSPISpeed() const {
    return spi_frequency;
}

uint8_t TFT7735V::getBrightness() const {
    return brightness_level;
}

uint8_t TFT7735V::getRotation() const {
    return rotation;
}

uint16_t TFT7735V::getWidth() const {
    return width;
}

uint16_t TFT7735V::getHeight() const {
    return height;
}

// Private helper functions
void TFT7735V::init_pwm() {
    if (bl_pin == GPIO_NUM_NC || pwm_initialized) {
        return;
    }
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT; // 8-bit resolution (0-255)
    ledc_timer.freq_hz = 5000; // 5kHz frequency
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        // Fallback to digital control
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << bl_pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        return;
    }
    
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.gpio_num = bl_pin;
    ledc_channel.duty = brightness_level;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        // Fallback to digital control
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << bl_pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        return;
    }
    
    pwm_initialized = true;
    ESP_LOGI(TAG, "PWM backlight control initialized");
}

void TFT7735V::apply_brightness() {
    if (bl_pin == GPIO_NUM_NC) {
        return;
    }
    
    if (pwm_initialized) {
        // Use PWM for smooth brightness control
        esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness_level);
        if (ret == ESP_OK) {
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        } else {
            ESP_LOGE(TAG, "Failed to set PWM duty: %s", esp_err_to_name(ret));
        }
    } else {
        // Fallback to digital control (on/off)
        gpio_set_level(bl_pin, brightness_level > 128 ? 1 : 0);
    }
}

void TFT7735V::update_spi_speed() {
    if (!initialized || !spi_device) {
        return;
    }
    
    // Note: ESP-IDF doesn't support changing SPI speed after device is added
    // We need to remove and re-add the device
    ESP_LOGI(TAG, "Updating SPI speed to: %lu Hz", spi_frequency);
    
    // Remove current device
    esp_err_t ret = spi_bus_remove_device(spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return;
    }
    
    // Re-add device with new speed
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = spi_frequency;
    devcfg.mode = 0; // SPI mode 0
    devcfg.spics_io_num = cs_pin;
    devcfg.queue_size = 7;
    devcfg.pre_cb = nullptr;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-add SPI device: %s", esp_err_to_name(ret));
        spi_device = nullptr;
        return;
    }
    
    ESP_LOGI(TAG, "SPI speed updated successfully");
}

// Framebuffer methods
bool TFT7735V::init_framebuffer() {
    if (framebuffer_a != nullptr || framebuffer_b != nullptr || framebuffer_c != nullptr) {
        ESP_LOGW(TAG, "Triple framebuffers already initialized");
        return true;
    }
    
    // Allocate triple framebuffers in PSRAM
    framebuffer_a = (uint16_t*)heap_caps_malloc(framebuffer_size, MALLOC_CAP_SPIRAM);
    if (framebuffer_a == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer A in PSRAM (%zu bytes)", framebuffer_size);
        return false;
    }
    
    framebuffer_b = (uint16_t*)heap_caps_malloc(framebuffer_size, MALLOC_CAP_SPIRAM);
    if (framebuffer_b == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer B in PSRAM (%zu bytes)", framebuffer_size);
        heap_caps_free(framebuffer_a);
        framebuffer_a = nullptr;
        return false;
    }
    
    framebuffer_c = (uint16_t*)heap_caps_malloc(framebuffer_size, MALLOC_CAP_SPIRAM);
    if (framebuffer_c == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer C in PSRAM (%zu bytes)", framebuffer_size);
        heap_caps_free(framebuffer_a);
        heap_caps_free(framebuffer_b);
        framebuffer_a = nullptr;
        framebuffer_b = nullptr;
        return false;
    }
    
    ESP_LOGI(TAG, "Triple framebuffers allocated in PSRAM (3 x %zu bytes = %zu total)", 
             framebuffer_size, framebuffer_size * 3);
    
    // Clear all framebuffers
    memset(framebuffer_a, 0, framebuffer_size);
    memset(framebuffer_b, 0, framebuffer_size);
    memset(framebuffer_c, 0, framebuffer_size);
    
    // Set initial render buffer (buffer A)
    current_framebuffer = framebuffer_a;
    buffer_states[0] = BUFFER_STATE_RENDERING;
    
    return true;
}

void TFT7735V::free_framebuffer() {
    if (framebuffer_a != nullptr) {
        heap_caps_free(framebuffer_a);
        framebuffer_a = nullptr;
    }
    if (framebuffer_b != nullptr) {
        heap_caps_free(framebuffer_b);
        framebuffer_b = nullptr;
    }
    if (framebuffer_c != nullptr) {
        heap_caps_free(framebuffer_c);
        framebuffer_c = nullptr;
    }
    current_framebuffer = nullptr;
    framebuffer_enabled = false;
    ESP_LOGI(TAG, "Triple framebuffers freed");
}

bool TFT7735V::enableFramebuffer() {
    if (framebuffer_enabled) {
        ESP_LOGW(TAG, "Framebuffer already enabled");
        return true;
    }
    
    if (!init_framebuffer()) {
        return false;
    }
    
    if (!init_double_buffering()) {
        free_framebuffer();
        return false;
    }
    
    framebuffer_enabled = true;
    ESP_LOGI(TAG, "Framebuffer mode with double buffering enabled");
    return true;
}

void TFT7735V::disableFramebuffer() {
    framebuffer_enabled = false;
    free_framebuffer();
    ESP_LOGI(TAG, "Framebuffer mode disabled");
}

bool TFT7735V::isFramebufferEnabled() const {
    return framebuffer_enabled;
}

void TFT7735V::display() {
    if (!framebuffer_enabled || current_framebuffer == nullptr) {
        ESP_LOGW(TAG, "Framebuffer not enabled or not allocated");
        return;
    }
    
    if (!initialized) {
        ESP_LOGE(TAG, "Display not initialized");
        return;
    }
    
    if (display_in_progress) {
        ESP_LOGW(TAG, "Display operation already in progress");
        return;
    }
    
    if (display_queue == nullptr) {
        ESP_LOGE(TAG, "Display queue not initialized");
        return;
    }
    
    // Perform buffer swap - find next available buffer for rendering
    uint8_t next_render_idx = 255; // Invalid index
    for (int i = 0; i < 3; i++) {
        if (buffer_states[i] == BUFFER_STATE_IDLE) {
            next_render_idx = i;
            break;
        }
    }
    
    if (next_render_idx == 255) {
        ESP_LOGW(TAG, "No idle buffer available for swap");
        return;
    }
    
    // Set current render buffer to transferring state
    buffer_states[render_buffer_idx] = BUFFER_STATE_TRANSFERRING;
    transfer_buffer_idx = render_buffer_idx;
    
    // Switch to next render buffer
    render_buffer_idx = next_render_idx;
    buffer_states[render_buffer_idx] = BUFFER_STATE_RENDERING;
    
    // Update current framebuffer pointer
    switch (render_buffer_idx) {
        case 0: current_framebuffer = framebuffer_a; break;
        case 1: current_framebuffer = framebuffer_b; break;
        case 2: current_framebuffer = framebuffer_c; break;
    }
    
    ESP_LOGI(TAG, "Buffer swap: render_idx=%d, transfer_idx=%d", render_buffer_idx, transfer_buffer_idx);
    ESP_LOGI(TAG, "Starting async display operation (%dx%d, %d chunks)", width, height, total_chunks);
    
    // Mark display as in progress
    display_in_progress = true;
    display_done_flag = false;
    current_chunk = 0;
      // Take semaphore to indicate display is not done
    xSemaphoreTake(display_done_semaphore, 0);
    
    // Determine if we should use dirty rectangle optimization
    bool use_dirty_rect = dirty_rect_enabled && dirty_rect.valid && !force_full_redraw;
    uint8_t start_chunk = 0, end_chunk = total_chunks - 1;
    uint8_t chunks_to_send = total_chunks;
    
    if (use_dirty_rect) {
        chunks_to_send = calculate_dirty_chunks(dirty_rect, start_chunk, end_chunk);
        ESP_LOGI(TAG, "Using dirty rect optimization: chunks %d-%d (%d chunks)", 
                 start_chunk, end_chunk, chunks_to_send);
    } else {
        ESP_LOGI(TAG, "Full frame display: %d chunks", total_chunks);
    }
    
    // Send first chunk message to display task
    display_message_t msg = {
        .chunk_idx = start_chunk,
        .is_last_chunk = (chunks_to_send == 1),
        .source_buffer_idx = transfer_buffer_idx,
        .use_dirty_rect = use_dirty_rect,
        .dirty_rect = use_dirty_rect ? dirty_rect : dirty_rect_t{0, 0, 0, 0, false}
    };
    
    BaseType_t ret = xQueueSend(display_queue, &msg, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to send display message to queue");
        display_in_progress = false;
        display_done_flag = true;
        xSemaphoreGive(display_done_semaphore);
        // Revert buffer states
        buffer_states[transfer_buffer_idx] = BUFFER_STATE_IDLE;
        return;
    }
    
    ESP_LOGD(TAG, "Display operation started with buffer %d", transfer_buffer_idx);
}

// Framebuffer drawing functions
void TFT7735V::fb_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= width || y >= height || current_framebuffer == nullptr) {
        return;
    }
    
    current_framebuffer[y * width + x] = color;
    
    // Track dirty rectangle
    expand_dirty_rect(x, y, 1, 1);
}

void TFT7735V::fb_fill_screen(uint16_t color) {
    if (current_framebuffer == nullptr) return;
    
    for (size_t i = 0; i < (width * height); i++) {
        current_framebuffer[i] = color;
    }
    
    // Full screen is dirty
    expand_dirty_rect(0, 0, width, height);
}

void TFT7735V::fb_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (current_framebuffer == nullptr) return;
    if (x >= width || y >= height) return;
    
    // Clip rectangle to screen bounds
    if (x + w > width) w = width - x;
    if (y + h > height) h = height - y;
    
    for (uint16_t row = y; row < y + h; row++) {
        for (uint16_t col = x; col < x + w; col++) {
            current_framebuffer[row * width + col] = color;
        }
    }
    
    // Track dirty rectangle
    expand_dirty_rect(x, y, w, h);
}

void TFT7735V::fb_draw_fast_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    fb_fill_rect(x, y, w, 1, color);
}

void TFT7735V::fb_draw_fast_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    fb_fill_rect(x, y, 1, h, color);
}

// New framebuffer drawing implementations
void TFT7735V::fb_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    if (current_framebuffer == nullptr) return;
    
    // Bresenham line algorithm for framebuffer
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx - dy;
    
    uint16_t min_x = std::min(x0, x1);
    uint16_t max_x = std::max(x0, x1);
    uint16_t min_y = std::min(y0, y1);
    uint16_t max_y = std::max(y0, y1);
    
    while (true) {
        if (x0 < width && y0 < height) {
            current_framebuffer[y0 * width + x0] = color;
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    // Track dirty rectangle
    if (max_x >= min_x && max_y >= min_y) {
        expand_dirty_rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    }
}

void TFT7735V::fb_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (current_framebuffer == nullptr || w == 0 || h == 0) return;
    
    // Draw rectangle outline using fast lines
    fb_draw_fast_hline(x, y, w, color);           // Top edge
    fb_draw_fast_hline(x, y + h - 1, w, color);  // Bottom edge
    if (h > 2) {
        fb_draw_fast_vline(x, y + 1, h - 2, color);         // Left edge
        fb_draw_fast_vline(x + w - 1, y + 1, h - 2, color); // Right edge
    }
}

void TFT7735V::fb_draw_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    if (current_framebuffer == nullptr) return;
    
    // Bresenham circle algorithm
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;
    
    while (x >= y) {
        // Draw 8 octants
        if (x0 + x < width && y0 + y < height) current_framebuffer[(y0 + y) * width + (x0 + x)] = color;
        if (x0 + y < width && y0 + x < height) current_framebuffer[(y0 + x) * width + (x0 + y)] = color;
        if (x0 - y < width && y0 + x < height && x0 >= y) current_framebuffer[(y0 + x) * width + (x0 - y)] = color;
        if (x0 - x < width && y0 + y < height && x0 >= x) current_framebuffer[(y0 + y) * width + (x0 - x)] = color;
        if (x0 - x < width && y0 - y < height && x0 >= x && y0 >= y) current_framebuffer[(y0 - y) * width + (x0 - x)] = color;
        if (x0 - y < width && y0 - x < height && x0 >= y && y0 >= x) current_framebuffer[(y0 - x) * width + (x0 - y)] = color;
        if (x0 + y < width && y0 - x < height && y0 >= x) current_framebuffer[(y0 - x) * width + (x0 + y)] = color;
        if (x0 + x < width && y0 - y < height && y0 >= y) current_framebuffer[(y0 - y) * width + (x0 + x)] = color;
        
        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
    
    // Track dirty rectangle - circle bounds
    uint16_t circle_x = (x0 >= r) ? x0 - r : 0;
    uint16_t circle_y = (y0 >= r) ? y0 - r : 0;
    uint16_t circle_w = (x0 + r < width) ? 2 * r + 1 : width - circle_x;
    uint16_t circle_h = (y0 + r < height) ? 2 * r + 1 : height - circle_y;
    expand_dirty_rect(circle_x, circle_y, circle_w, circle_h);
}

void TFT7735V::fb_fill_circle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    if (current_framebuffer == nullptr) return;
    
    // Filled circle using horizontal lines
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;
    
    while (x >= y) {
        // Draw horizontal lines for fill
        if (y0 + y < height) {
            uint16_t start_x = (x0 >= x) ? x0 - x : 0;
            uint16_t end_x = (x0 + x < width) ? x0 + x : width - 1;
            for (uint16_t px = start_x; px <= end_x; px++) {
                current_framebuffer[(y0 + y) * width + px] = color;
            }
        }
        
        if (y0 - y < height && y0 >= y && y != 0) {
            uint16_t start_x = (x0 >= x) ? x0 - x : 0;
            uint16_t end_x = (x0 + x < width) ? x0 + x : width - 1;
            for (uint16_t px = start_x; px <= end_x; px++) {
                current_framebuffer[(y0 - y) * width + px] = color;
            }
        }
        
        if (y0 + x < height && x != y) {
            uint16_t start_x = (x0 >= y) ? x0 - y : 0;
            uint16_t end_x = (x0 + y < width) ? x0 + y : width - 1;
            for (uint16_t px = start_x; px <= end_x; px++) {
                current_framebuffer[(y0 + x) * width + px] = color;
            }
        }
        
        if (y0 - x < height && y0 >= x && x != 0 && x != y) {
            uint16_t start_x = (x0 >= y) ? x0 - y : 0;
            uint16_t end_x = (x0 + y < width) ? x0 + y : width - 1;
            for (uint16_t px = start_x; px <= end_x; px++) {
                current_framebuffer[(y0 - x) * width + px] = color;
            }
        }
        
        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
    
    // Track dirty rectangle - circle bounds
    uint16_t circle_x = (x0 >= r) ? x0 - r : 0;
    uint16_t circle_y = (y0 >= r) ? y0 - r : 0;
    uint16_t circle_w = (x0 + r < width) ? 2 * r + 1 : width - circle_x;
    uint16_t circle_h = (y0 + r < height) ? 2 * r + 1 : height - circle_y;
    expand_dirty_rect(circle_x, circle_y, circle_w, circle_h);
}

void TFT7735V::fb_draw_bitmap(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t w, uint16_t h, uint16_t color, uint16_t bg, bool has_bg) {
    if (current_framebuffer == nullptr || bitmap == nullptr) return;
    
    for (uint16_t row = 0; row < h; row++) {
        if (y + row >= height) break;
        
        for (uint16_t col = 0; col < w; col++) {
            if (x + col >= width) break;
            
            uint16_t byte_idx = (row * ((w + 7) / 8)) + (col / 8);
            uint8_t bit_idx = 7 - (col % 8);
            bool pixel_set = bitmap[byte_idx] & (1 << bit_idx);
            
            if (pixel_set) {
                current_framebuffer[(y + row) * width + (x + col)] = color;
            } else if (has_bg) {
                current_framebuffer[(y + row) * width + (x + col)] = bg;
            }
        }
    }
    
    // Track dirty rectangle
    uint16_t clipped_w = (x + w > width) ? width - x : w;
    uint16_t clipped_h = (y + h > height) ? height - y : h;
    expand_dirty_rect(x, y, clipped_w, clipped_h);
}

void TFT7735V::fb_draw_rgb_bitmap(uint16_t x, uint16_t y, const uint16_t *bitmap, const uint8_t *mask, uint16_t w, uint16_t h, bool has_mask) {
    if (current_framebuffer == nullptr || bitmap == nullptr) return;
    
    for (uint16_t row = 0; row < h; row++) {
        if (y + row >= height) break;
        
        for (uint16_t col = 0; col < w; col++) {
            if (x + col >= width) break;
            
            bool draw_pixel = true;
            if (has_mask && mask != nullptr) {
                uint16_t mask_byte_idx = (row * ((w + 7) / 8)) + (col / 8);
                uint8_t mask_bit_idx = 7 - (col % 8);
                draw_pixel = mask[mask_byte_idx] & (1 << mask_bit_idx);
            }
            
            if (draw_pixel) {
                uint16_t pixel_color = bitmap[row * w + col];
                current_framebuffer[(y + row) * width + (x + col)] = pixel_color;
            }
        }
    }
    
    // Track dirty rectangle
    uint16_t clipped_w = (x + w > width) ? width - x : w;
    uint16_t clipped_h = (y + h > height) ? height - y : h;
    expand_dirty_rect(x, y, clipped_w, clipped_h);
}

void TFT7735V::fb_draw_char(uint16_t x, uint16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size, bool has_bg) {
    if (current_framebuffer == nullptr) return;
    
    // Bounds checking
    if (x >= width || y >= height) return;
    
    // Character validation
    if (c < FONT8X8_FIRST_CHAR || c > FONT8X8_LAST_CHAR) {
        c = '?'; // Default character for unsupported chars
    }
    
    const uint8_t* char_data = font8x8_basic[c - FONT8X8_FIRST_CHAR];
      // Draw character bitmap with scaling
    for (int8_t row = 0; row < FONT8X8_HEIGHT; row++) {
        uint8_t line = char_data[row];
        for (int8_t col = 0; col < FONT8X8_WIDTH; col++) {
            bool pixel_set = line & (0x01 << col);
            
            // Draw scaled pixel
            for (uint8_t sy = 0; sy < size; sy++) {
                for (uint8_t sx = 0; sx < size; sx++) {
                    uint16_t px = x + col * size + sx;
                    uint16_t py = y + row * size + sy;
                    
                    if (px < width && py < height) {
                        if (pixel_set) {
                            current_framebuffer[py * width + px] = color;
                        } else if (has_bg) {
                            current_framebuffer[py * width + px] = bg;
                        }
                    }
                }
            }
        }
    }
    
    // Track dirty rectangle
    uint16_t char_width = FONT8X8_WIDTH * size;
    uint16_t char_height = FONT8X8_HEIGHT * size;
    uint16_t clipped_w = (x + char_width > width) ? width - x : char_width;
    uint16_t clipped_h = (y + char_height > height) ? height - y : char_height;
    expand_dirty_rect(x, y, clipped_w, clipped_h);
}

// Double buffering methods
bool TFT7735V::init_double_buffering() {
    if (sram_buffer_a != nullptr || sram_buffer_b != nullptr) {
        ESP_LOGW(TAG, "Double buffering already initialized");
        return true;
    }
    
    // Allocate SRAM buffers (8KB each)
    sram_buffer_a = (uint16_t*)heap_caps_malloc(SRAM_BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    if (sram_buffer_a == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SRAM buffer A (%d bytes)", SRAM_BUFFER_SIZE);
        return false;
    }
    
    sram_buffer_b = (uint16_t*)heap_caps_malloc(SRAM_BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    if (sram_buffer_b == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate SRAM buffer B (%d bytes)", SRAM_BUFFER_SIZE);
        heap_caps_free(sram_buffer_a);
        sram_buffer_a = nullptr;
        return false;
    }
    
    // Create display queue
    display_queue = xQueueCreate(10, sizeof(display_message_t));
    if (display_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create display queue");
        heap_caps_free(sram_buffer_a);
        heap_caps_free(sram_buffer_b);
        sram_buffer_a = nullptr;
        sram_buffer_b = nullptr;
        return false;
    }
    
    // Create semaphore for display done notification
    display_done_semaphore = xSemaphoreCreateBinary();
    if (display_done_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create display done semaphore");
        vQueueDelete(display_queue);
        heap_caps_free(sram_buffer_a);
        heap_caps_free(sram_buffer_b);
        sram_buffer_a = nullptr;
        sram_buffer_b = nullptr;
        display_queue = nullptr;
        return false;
    }
    
    // Create display task
    BaseType_t ret = xTaskCreate(display_task, "display_task", 4096, this, 5, &display_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        vSemaphoreDelete(display_done_semaphore);
        vQueueDelete(display_queue);
        heap_caps_free(sram_buffer_a);
        heap_caps_free(sram_buffer_b);
        sram_buffer_a = nullptr;
        sram_buffer_b = nullptr;
        display_queue = nullptr;
        display_done_semaphore = nullptr;
        return false;
    }
    
    // Set initial buffer state
    current_sram_buffer = sram_buffer_a;
    display_in_progress = false;
    display_done_flag = true;
    current_chunk = 0;
    
    // Give semaphore initially (display is "done")
    xSemaphoreGive(display_done_semaphore);
    
    ESP_LOGI(TAG, "Double buffering initialized: SRAM buffers %d bytes each, %d chunks per frame", 
             SRAM_BUFFER_SIZE, total_chunks);
    ESP_LOGI(TAG, "Chunk height: %d pixels, Total chunks: %d", CHUNK_HEIGHT, total_chunks);
    
    return true;
}

void TFT7735V::free_double_buffering() {
    // Wait for any ongoing display operation to complete
    if (display_in_progress) {
        ESP_LOGI(TAG, "Waiting for display operation to complete before freeing buffers...");
        waitForDisplayDone();
    }
    
    // Delete display task
    if (display_task_handle != nullptr) {
        vTaskDelete(display_task_handle);
        display_task_handle = nullptr;
    }
    
    if (display_done_semaphore != nullptr) {
        vSemaphoreDelete(display_done_semaphore);
        display_done_semaphore = nullptr;
    }
    
    if (display_queue != nullptr) {
        vQueueDelete(display_queue);
        display_queue = nullptr;
    }
    
    if (sram_buffer_a != nullptr) {
        heap_caps_free(sram_buffer_a);
        sram_buffer_a = nullptr;
    }
    
    if (sram_buffer_b != nullptr) {
        heap_caps_free(sram_buffer_b);
        sram_buffer_b = nullptr;
    }
    
    current_sram_buffer = nullptr;
    display_in_progress = false;
    display_done_flag = true;
    
    ESP_LOGI(TAG, "Double buffering freed");
}

// Display task function
void TFT7735V::display_task(void* pvParameters) {
    TFT7735V* tft = (TFT7735V*)pvParameters;
    display_message_t msg;
    
    ESP_LOGI(TAG, "Display task started");
    
    while (true) {        // Wait for display message
        if (xQueueReceive(tft->display_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "Processing chunk %d from buffer %d, last=%d, dirty=%d", 
                     msg.chunk_idx, msg.source_buffer_idx, msg.is_last_chunk, msg.use_dirty_rect);
            
            // Process the chunk from the specified source buffer
            if (msg.use_dirty_rect && msg.dirty_rect.valid) {
                tft->copy_dirty_chunk_and_send(msg.chunk_idx, msg.source_buffer_idx, msg.dirty_rect);
            } else {
                tft->copy_chunk_and_send(msg.chunk_idx, msg.source_buffer_idx);
            }
            
            if (msg.is_last_chunk) {
                // Mark source buffer as idle now that transfer is complete
                tft->buffer_states[msg.source_buffer_idx] = BUFFER_STATE_IDLE;
                
                // Clear dirty rectangle after successful transfer
                tft->clearDirty();
                
                // Mark display as completed
                tft->display_in_progress = false;
                tft->display_done_flag = true;
                xSemaphoreGive(tft->display_done_semaphore);
                ESP_LOGI(TAG, "Display operation completed, buffer %d now idle", msg.source_buffer_idx);            } else {
                // Calculate next chunk for dirty rect mode
                uint8_t next_chunk_idx;
                bool is_last;
                
                if (msg.use_dirty_rect && msg.dirty_rect.valid) {
                    // In dirty rect mode, calculate end chunk
                    uint8_t start_chunk, end_chunk;
                    tft->calculate_dirty_chunks(msg.dirty_rect, start_chunk, end_chunk);
                    next_chunk_idx = msg.chunk_idx + 1;
                    is_last = (next_chunk_idx > end_chunk);
                } else {
                    // Full frame mode
                    next_chunk_idx = msg.chunk_idx + 1;
                    is_last = (next_chunk_idx >= tft->total_chunks);
                }
                
                if (!is_last) {
                    // Send next chunk message
                    display_message_t next_msg = {
                        .chunk_idx = next_chunk_idx,
                        .is_last_chunk = is_last,
                        .source_buffer_idx = msg.source_buffer_idx,
                        .use_dirty_rect = msg.use_dirty_rect,
                        .dirty_rect = msg.dirty_rect
                    };
                    
                    if (xQueueSend(tft->display_queue, &next_msg, 0) != pdTRUE) {
                        ESP_LOGE(TAG, "Failed to send next chunk message");
                        // Mark buffer as idle on error
                        tft->buffer_states[msg.source_buffer_idx] = BUFFER_STATE_IDLE;
                        tft->display_in_progress = false;
                        tft->display_done_flag = true;
                        xSemaphoreGive(tft->display_done_semaphore);
                    }
                } else {
                    // This was actually the last chunk
                    tft->buffer_states[msg.source_buffer_idx] = BUFFER_STATE_IDLE;
                    tft->clearDirty();
                    tft->display_in_progress = false;
                    tft->display_done_flag = true;
                    xSemaphoreGive(tft->display_done_semaphore);
                    ESP_LOGI(TAG, "Display operation completed, buffer %d now idle", msg.source_buffer_idx);
                }
            }
        }
    }
}

void TFT7735V::copy_chunk_and_send(uint8_t chunk_idx, uint8_t source_buffer_idx) {
    if (!initialized) {
        return;
    }
    
    // Get source framebuffer based on buffer index
    uint16_t* source_framebuffer = nullptr;
    switch (source_buffer_idx) {
        case 0: source_framebuffer = framebuffer_a; break;
        case 1: source_framebuffer = framebuffer_b; break;
        case 2: source_framebuffer = framebuffer_c; break;
        default:
            ESP_LOGE(TAG, "Invalid source buffer index: %d", source_buffer_idx);
            return;
    }
    
    if (source_framebuffer == nullptr) {
        ESP_LOGE(TAG, "Source framebuffer %d is null", source_buffer_idx);
        return;
    }
    
    // Calculate chunk dimensions
    uint16_t chunk_start_y = chunk_idx * CHUNK_HEIGHT;
    uint16_t chunk_end_y = (chunk_idx + 1) * CHUNK_HEIGHT;
    if (chunk_end_y > height) {
        chunk_end_y = height;
    }
    
    uint16_t actual_chunk_height = chunk_end_y - chunk_start_y;
    
    ESP_LOGD(TAG, "Processing chunk %d from buffer %d: y=%d-%d, height=%d", 
             chunk_idx, source_buffer_idx, chunk_start_y, chunk_end_y, actual_chunk_height);
    
    // Use alternating SRAM buffers for double buffering
    uint16_t* target_buffer = (chunk_idx % 2 == 0) ? sram_buffer_a : sram_buffer_b;
    
    // Copy chunk from PSRAM framebuffer to SRAM buffer
    uint16_t* src = source_framebuffer + (chunk_start_y * width);
    size_t chunk_size_pixels = width * actual_chunk_height;
    
    // Fast memcpy from PSRAM to SRAM
    memcpy(target_buffer, src, chunk_size_pixels * sizeof(uint16_t));
    
    // Send to display
    send_chunk_to_display(target_buffer, chunk_idx, actual_chunk_height);
}

void TFT7735V::send_chunk_to_display(uint16_t* buffer, uint16_t chunk_idx, uint16_t chunk_height) {
    if (!initialized || buffer == nullptr) {
        return;
    }
    
    // Calculate chunk position
    uint16_t chunk_start_y = chunk_idx * CHUNK_HEIGHT;
    
    ESP_LOGD(TAG, "Sending chunk %d to display: y=%d, height=%d", 
             chunk_idx, chunk_start_y, chunk_height);
    
    // Set address window for this chunk
    set_addr_window(0, chunk_start_y, width - 1, chunk_start_y + chunk_height - 1);
    
    // Convert endianness and send via SPI
    gpio_set_level(dc_pin, 1); // Data mode
    
    size_t total_pixels = width * chunk_height;
    const size_t spi_chunk_size = 2048; // Pixels per SPI transaction
    
    for (size_t pixel_offset = 0; pixel_offset < total_pixels; pixel_offset += spi_chunk_size) {
        size_t current_pixels = (total_pixels - pixel_offset > spi_chunk_size) ? 
                               spi_chunk_size : (total_pixels - pixel_offset);
        
        // Convert endianness in-place (temporarily)
        for (size_t i = 0; i < current_pixels; i++) {
            buffer[pixel_offset + i] = __builtin_bswap16(buffer[pixel_offset + i]);
        }
        
        // Send via SPI
        spi_transaction_t t = {};
        t.length = current_pixels * 16; // Length in bits
        t.tx_buffer = &buffer[pixel_offset];
        
        esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit chunk %d: %s", chunk_idx, esp_err_to_name(ret));
            return;
        }
        
        // Restore original endianness
        for (size_t i = 0; i < current_pixels; i++) {
            buffer[pixel_offset + i] = __builtin_bswap16(buffer[pixel_offset + i]);
        }
    }
}

// Public methods for display status
bool TFT7735V::displayDone() const {
    return display_done_flag;
}

void TFT7735V::waitForDisplayDone() {
    if (display_done_semaphore != nullptr && display_in_progress) {
        ESP_LOGD(TAG, "Waiting for display operation to complete...");
        xSemaphoreTake(display_done_semaphore, portMAX_DELAY);
        xSemaphoreGive(display_done_semaphore); // Give it back immediately
    }
}

void TFT7735V::swapBuffers() {
    if (!framebuffer_enabled) {
        ESP_LOGW(TAG, "Framebuffer not enabled");
        return;
    }
    
    // Find next available buffer for rendering
    uint8_t next_render_idx = 255; // Invalid index
    for (int i = 0; i < 3; i++) {
        if (buffer_states[i] == BUFFER_STATE_IDLE) {
            next_render_idx = i;
            break;
        }
    }
    
    if (next_render_idx == 255) {
        ESP_LOGW(TAG, "No idle buffer available for manual swap");
        return;
    }
    
    // Switch to next render buffer
    uint8_t old_render_idx = render_buffer_idx;
    render_buffer_idx = next_render_idx;
    buffer_states[render_buffer_idx] = BUFFER_STATE_RENDERING;
    
    // Previous render buffer becomes idle (ready for display)
    buffer_states[old_render_idx] = BUFFER_STATE_IDLE;
    
    // Update current framebuffer pointer
    switch (render_buffer_idx) {
        case 0: current_framebuffer = framebuffer_a; break;
        case 1: current_framebuffer = framebuffer_b; break;
        case 2: current_framebuffer = framebuffer_c; break;
    }
    
    ESP_LOGI(TAG, "Manual buffer swap: %d -> %d", old_render_idx, render_buffer_idx);
}

// Dirty rectangle optimization implementation
void TFT7735V::enableDirtyRect(bool enable) {
    dirty_rect_enabled = enable;
    if (enable) {
        // Clear dirty rect when enabling
        clearDirty();
        ESP_LOGI(TAG, "Dirty rectangle optimization enabled");
    } else {
        ESP_LOGI(TAG, "Dirty rectangle optimization disabled");
    }
}

void TFT7735V::clearDirty() {
    dirty_rect.valid = false;
    dirty_rect.x = 0;
    dirty_rect.y = 0;
    dirty_rect.w = 0;
    dirty_rect.h = 0;
    force_full_redraw = false;
}

void TFT7735V::forceFullRedraw() {
    force_full_redraw = true;
    dirty_rect.valid = false;  // Invalid dirty rect forces full redraw
    ESP_LOGI(TAG, "Full redraw forced");
}

bool TFT7735V::isDirtyRectEnabled() const {
    return dirty_rect_enabled;
}

void TFT7735V::expand_dirty_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!dirty_rect_enabled) {
        return;
    }
    
    // Clip to screen bounds
    if (x >= width || y >= height) return;
    if (x + w > width) w = width - x;
    if (y + h > height) h = height - y;
    if (w == 0 || h == 0) return;
    
    if (!dirty_rect.valid) {
        // First dirty region
        dirty_rect.x = x;
        dirty_rect.y = y;
        dirty_rect.w = w;
        dirty_rect.h = h;
        dirty_rect.valid = true;
    } else {
        // Expand existing dirty rectangle to include new region
        uint16_t x1 = std::min(dirty_rect.x, x);
        uint16_t y1 = std::min(dirty_rect.y, y);
        uint16_t x2 = std::max(dirty_rect.x + dirty_rect.w, x + w);
        uint16_t y2 = std::max(dirty_rect.y + dirty_rect.h, y + h);
        
        dirty_rect.x = x1;
        dirty_rect.y = y1;
        dirty_rect.w = x2 - x1;
        dirty_rect.h = y2 - y1;
    }
    
    ESP_LOGD(TAG, "Dirty rect expanded to (%d,%d) %dx%d", 
             dirty_rect.x, dirty_rect.y, dirty_rect.w, dirty_rect.h);
}

uint8_t TFT7735V::calculate_dirty_chunks(const dirty_rect_t& dirty_rect, uint8_t& start_chunk, uint8_t& end_chunk) {
    if (!dirty_rect.valid) {
        start_chunk = 0;
        end_chunk = total_chunks - 1;
        return total_chunks;
    }
    
    // Calculate which chunks are affected by dirty rectangle
    start_chunk = dirty_rect.y / CHUNK_HEIGHT;
    end_chunk = (dirty_rect.y + dirty_rect.h - 1) / CHUNK_HEIGHT;
    
    // Ensure chunks are within bounds
    if (start_chunk >= total_chunks) start_chunk = total_chunks - 1;
    if (end_chunk >= total_chunks) end_chunk = total_chunks - 1;
    
    uint8_t affected_chunks = end_chunk - start_chunk + 1;
    
    ESP_LOGD(TAG, "Dirty rect (%d,%d) %dx%d affects chunks %d-%d (%d chunks)", 
             dirty_rect.x, dirty_rect.y, dirty_rect.w, dirty_rect.h,
             start_chunk, end_chunk, affected_chunks);
    
    return affected_chunks;
}

void TFT7735V::copy_dirty_chunk_and_send(uint8_t chunk_idx, uint8_t source_buffer_idx, const dirty_rect_t& dirty_rect) {
    if (!initialized) {
        return;
    }
    
    // Get source framebuffer based on buffer index
    uint16_t* source_framebuffer = nullptr;
    switch (source_buffer_idx) {
        case 0: source_framebuffer = framebuffer_a; break;
        case 1: source_framebuffer = framebuffer_b; break;
        case 2: source_framebuffer = framebuffer_c; break;
        default:
            ESP_LOGE(TAG, "Invalid source buffer index: %d", source_buffer_idx);
            return;
    }
    
    if (source_framebuffer == nullptr) {
        ESP_LOGE(TAG, "Source framebuffer %d is null", source_buffer_idx);
        return;
    }
    
    // Calculate chunk dimensions
    uint16_t chunk_start_y = chunk_idx * CHUNK_HEIGHT;
    uint16_t chunk_end_y = (chunk_idx + 1) * CHUNK_HEIGHT;
    if (chunk_end_y > height) {
        chunk_end_y = height;
    }
    
    uint16_t actual_chunk_height = chunk_end_y - chunk_start_y;
    
    // Calculate intersection of dirty rect with this chunk
    uint16_t dirty_start_y = std::max(dirty_rect.y, chunk_start_y);
    uint16_t dirty_end_y = std::min((uint16_t)(dirty_rect.y + dirty_rect.h), chunk_end_y);
    
    if (dirty_start_y >= dirty_end_y) {
        // No intersection - chunk is not dirty
        ESP_LOGD(TAG, "Chunk %d not dirty, skipping", chunk_idx);
        return;
    }
    
    uint16_t dirty_height = dirty_end_y - dirty_start_y;
    uint16_t dirty_x = dirty_rect.x;
    uint16_t dirty_w = dirty_rect.w;
    
    ESP_LOGD(TAG, "Processing dirty chunk %d from buffer %d: dirty region (%d,%d) %dx%d", 
             chunk_idx, source_buffer_idx, dirty_x, dirty_start_y, dirty_w, dirty_height);
    
    // Use alternating SRAM buffers for double buffering
    uint16_t* target_buffer = (chunk_idx % 2 == 0) ? sram_buffer_a : sram_buffer_b;
    
    // Copy dirty region from PSRAM framebuffer to SRAM buffer
    // We need to copy full width chunks for efficient SPI transfer
    uint16_t* src = source_framebuffer + (chunk_start_y * width);
    size_t chunk_size_pixels = width * actual_chunk_height;
    
    // Fast memcpy from PSRAM to SRAM (full chunk for simplicity)
    memcpy(target_buffer, src, chunk_size_pixels * sizeof(uint16_t));
    
    // Send to display with dirty rectangle info
    send_dirty_chunk_to_display(target_buffer, chunk_idx, actual_chunk_height, dirty_rect);
}

void TFT7735V::send_dirty_chunk_to_display(uint16_t* buffer, uint16_t chunk_idx, uint16_t chunk_height, const dirty_rect_t& dirty_rect) {
    if (!initialized || buffer == nullptr) {
        return;
    }
    
    // Calculate chunk position
    uint16_t chunk_start_y = chunk_idx * CHUNK_HEIGHT;
    
    // Calculate intersection of dirty rect with this chunk
    uint16_t dirty_start_y = std::max(dirty_rect.y, chunk_start_y);
    uint16_t dirty_end_y = std::min((uint16_t)(dirty_rect.y + dirty_rect.h), (uint16_t)(chunk_start_y + chunk_height));
    
    if (dirty_start_y >= dirty_end_y) {
        ESP_LOGD(TAG, "Chunk %d not dirty, skipping display", chunk_idx);
        return;
    }
    
    uint16_t dirty_height = dirty_end_y - dirty_start_y;
    uint16_t dirty_x = dirty_rect.x;
    uint16_t dirty_w = dirty_rect.w;
    
    ESP_LOGD(TAG, "Sending dirty chunk %d to display: region (%d,%d) %dx%d", 
             chunk_idx, dirty_x, dirty_start_y, dirty_w, dirty_height);
    
    // Set address window for dirty region only
    set_addr_window(dirty_x, dirty_start_y, dirty_x + dirty_w - 1, dirty_start_y + dirty_height - 1);
    
    // Convert endianness and send via SPI
    gpio_set_level(dc_pin, 1); // Data mode
    
    // Calculate offset in buffer for dirty region
    uint16_t dirty_offset_y = dirty_start_y - chunk_start_y;
    
    // Send dirty region line by line
    for (uint16_t row = 0; row < dirty_height; row++) {
        uint16_t* line_start = buffer + ((dirty_offset_y + row) * width) + dirty_x;
        
        // Convert endianness in-place (temporarily)
        for (uint16_t col = 0; col < dirty_w; col++) {
            line_start[col] = __builtin_bswap16(line_start[col]);
        }
        
        // Send line via SPI
        spi_transaction_t t = {};
        t.length = dirty_w * 16; // Length in bits
        t.tx_buffer = line_start;
        
        esp_err_t ret = spi_device_polling_transmit(spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit dirty line %d: %s", row, esp_err_to_name(ret));
            // Restore endianness before returning
            for (uint16_t col = 0; col < dirty_w; col++) {
                line_start[col] = __builtin_bswap16(line_start[col]);
            }
            return;
        }
        
        // Restore original endianness
        for (uint16_t col = 0; col < dirty_w; col++) {
            line_start[col] = __builtin_bswap16(line_start[col]);
        }
    }
}
