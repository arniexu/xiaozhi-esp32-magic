#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <driver/uart.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;
    // a variable to hold the current frame of uart sending
    uint8_t current_frame_[12] = {0};
    // a variable to hold the position of the current key in matrix key frame
    uint8_t current_mode_[2] = {'w', '1'};
    // a async timer that will be used to send the current frame
    esp_timer_handle_t send_frame_timer_ = nullptr;
    // initialize the timer for sending frame, every 100ms
    void InitializeSendFrameTimer() {
                esp_timer_create_args_t timer_args = {
                    .callback = [](void* arg) {
                        CompactWifiBoardLCD* board = static_cast<CompactWifiBoardLCD*>(arg);
                        if (board->send_frame_timer_) {
                            uart_write_bytes(UART_NUM_1, reinterpret_cast<const char*>(board->current_frame_), 12);
                            ESP_LOGI(TAG, "SendMatrixKeyFrame: "
                                "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                                board->current_frame_[0], board->current_frame_[1], board->current_frame_[2], board->current_frame_[3], board->current_frame_[4], board->current_frame_[5],
                                board->current_frame_[6], board->current_frame_[7], board->current_frame_[8], board->current_frame_[9], board->current_frame_[10], board->current_frame_[11]);
                            }
                        },
                    .arg = this,
                    .dispatch_method = ESP_TIMER_TASK,
                    .name = "SendFrameTimer"
                };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &send_frame_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(send_frame_timer_, 300 * 1000)); // 100ms
    }
    /** 
     * @brief 发送一帧矩阵键盘协议数据到UART2
     * 
     * @param row_char   行标字符（如 'W', 'D', 'F', 'R'，如未用可填 0）
     * @param col_char   列标字符（如 '1', '2', '3', '4'，如未用可填 0）
     * @param dir_char   方向键标识（如 's', 'a', 'd', 'w'，如未用可填 0）
     * @param beep      是否蜂鸣（true/false，true时第7字节为0x42，false为0x00）
     * @note  其余协议内容固定，校验自动计算
     */
    void SendMatrixKeyFrame(char row_char, char col_char, char dir_char, bool beep = true) {
        //把下面的frame替换成current_frame_
        current_frame_[0] = 0x56; // 帧头 'V'
        current_frame_[1] = 0x31; // 帧头 '1'
        current_frame_[2] = 0x8; // 类型
        current_frame_[3] = static_cast<uint8_t>(row_char); // 行标
        current_frame_[4] = static_cast<uint8_t>(col_char); // 列标
        current_frame_[5] = static_cast<uint8_t>(dir_char); // 方向键标识me_
        current_frame_[6] = beep ? 0x42 : 0x00;             // 蜂鸣器选择字符
        current_frame_[7] = 0x00;
        current_frame_[8] = 0x00;
        current_frame_[9] = 0x00;
        current_frame_[10] = 0x00;
        // 校验和：第4-11字节的和，取最低8位
        uint8_t checksum = 0;
        for (int i = 2; i <= 10; ++i) {
            checksum += current_frame_[i];
        }
        current_frame_[11] = checksum;

    }

    /**
     * @brief 发送默认空闲帧（无按键按下，协议内容固定）
     */
    void SendMatrixIdleFrame() {
        // 默认帧内容：56 31 08 57 31 73 42 00 00 00 00 45
        // 即 row='W', col='1', dir='s', beep=0x42
        SendMatrixKeyFrame('W', '1', 's', true);
    }

    void forward() {
        SendMatrixKeyFrame('W', '1', 'f', true);
    }

    // D  dance mode
    // Y dance 2 mode 
    // w walke mode
    // F fight mode
    // Z fight 2 mode
    // R record mode
    // L leg mode
    // T trim mode
    // G gait mode
    void switchmode(int mode) {
        switch(mode){
            case 0: SendMatrixKeyFrame('W', '1', 's', true); break;
            case 1: SendMatrixKeyFrame('W', '2', 's', true); break;
            case 2: SendMatrixKeyFrame('W', '3', 's', true); break;
            case 3: SendMatrixKeyFrame('W', '4', 's', true); break;
            case 4: SendMatrixKeyFrame('D', '1', 's', true); break;
            case 5: SendMatrixKeyFrame('D', '2', 's', true); break;
            case 6: SendMatrixKeyFrame('D', '3', 's', true); break;
            // help me with the rest 
            case 7: SendMatrixKeyFrame('D', '4', 's', true); break;
            case 8: SendMatrixKeyFrame('F', '1', 's', true); break;
            case 9: SendMatrixKeyFrame('F', '2', 's', true); break;
            case 10: SendMatrixKeyFrame('F', '3', 's', true); break;
            case 11: SendMatrixKeyFrame('F', '4', 's', true); break;
            case 12: SendMatrixKeyFrame('R', '1', 's', true); break;
            case 13: SendMatrixKeyFrame('R', '2', 's', true); break;
            case 14: SendMatrixKeyFrame('R', '3', 's', true); break;
            case 15: SendMatrixKeyFrame('R', '4', 's', true); break;
            // case 16-19 , 行标是X，列标依次是1-4
            case 16: SendMatrixKeyFrame('X', '1', 's', true); break;
            case 17: SendMatrixKeyFrame('X', '2', 's', true); break;
            case 18: SendMatrixKeyFrame('X', '3', 's', true); break;
            case 19: SendMatrixKeyFrame('X', '4', 's', true); break;
            // case 20-23 , 行标是Y，列标依次是1-4
            case 20: SendMatrixKeyFrame('Y', '1', 's', true); break;
            case 21: SendMatrixKeyFrame('Y', '2', 's', true); break;
            case 22: SendMatrixKeyFrame('Y', '3', 's', true); break;
            case 23: SendMatrixKeyFrame('Y', '4', 's', true); break;
            // case 24-27 , 行标是Z，列标依次是1-4
            case 24: SendMatrixKeyFrame('Z', '1', 's', true); break;
            case 25: SendMatrixKeyFrame('Z', '2', 's', true); break;
            case 26: SendMatrixKeyFrame('Z', '3', 's', true); break;
            case 27: SendMatrixKeyFrame('Z', '4', 's', true); break;
            // case 28-31 , 行标是L，列标依次是1-4
            case 28: SendMatrixKeyFrame('L', '1', 's', true); break;
            case 29: SendMatrixKeyFrame('L', '2', 's', true); break;
            case 30: SendMatrixKeyFrame('L', '3', 's', true); break;
            case 31: SendMatrixKeyFrame('L', '4', 's', true); break;
            // case 32-35 , 行标是T，列标依次是1-4
            case 32: SendMatrixKeyFrame('T', '1', 's', true); break;
            case 33: SendMatrixKeyFrame('T', '2', 's', true); break;
            case 34: SendMatrixKeyFrame('T', '3', 's', true); break;
            case 35: SendMatrixKeyFrame('T', '4', 's', true); break;
            default: break;
        }
    }

    // type is balley, finger, dab, wave
    // duration is in seconds
    void dance(char *type, int duration) {
        // 发送舞蹈指令
        // 这里假设 type 是一个字符数组，包含舞蹈类型
        // duration 是舞蹈持续时间，单位为秒
        // 例如：type = "balley", duration = 5
        // 启动一个新的局部一次性的定时器，使用定时器每隔100ms发送跳舞指令
        // 定时器时间到了销毁
        if (duration <= 0) {
            ESP_LOGE(TAG, "Invalid duration for dance: %d seconds", duration);
            return;
        }
        if (type == nullptr || strlen(type) == 0) {
            ESP_LOGE(TAG, "Invalid dance type");
            return;
        }
        // 不同舞蹈类型发送不同的控制帧
        if ( strcmp(type,"dab") == 0 ) {
            SendMatrixKeyFrame('D', '1', 'b', true); // 假设 'D' 是舞蹈类型
        } else if (strcmp(type, "balley") == 0) {
            SendMatrixKeyFrame('D', '2', 'f', true);
        } else if (strcmp(type, "finger") == 0) {
            SendMatrixKeyFrame('D', '3', 'd', true);
        } else if (strcmp(type, "wave") == 0) {
            SendMatrixKeyFrame('D', '4', 'w', true);
        } else {
            ESP_LOGE(TAG, "Unknown dance type: %s", type);
            return;
        }
        // 创建一个一次性定时器，定时器到期后发送空闲帧
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                CompactWifiBoardLCD* board = static_cast<CompactWifiBoardLCD*>(arg);
                if (board == nullptr) {
                    ESP_LOGE(TAG, "Dance timer callback received null board pointer");
                    return;
                }
                board->SendMatrixIdleFrame(); // 发送空闲帧
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "DanceTimer"
        };
        esp_timer_handle_t dance_timer;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &dance_timer));
        ESP_ERROR_CHECK(esp_timer_start_once(dance_timer, duration * 1000*1000)); // 转换为微秒
        ESP_LOGI(TAG, "Dance type: %s, duration: %d seconds", type, duration);
        return;
    }

    void backward() {
        SendMatrixKeyFrame('W', '1', 'b', true);
    }

    void left() {
        SendMatrixKeyFrame('W', '1', 'l', true);
    }

    void right() {
        SendMatrixKeyFrame('W', '1', 'r', true);
    }

    void stop() {
        SendMatrixIdleFrame();
    }

    // 串口初始化（如有需要可在构造函数或初始化流程中调用）
    void InitializeUart() {
        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_38, GPIO_NUM_48, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // TX=17, RX=16
    }

    // 串口发送字符串
    void SendUartMessage(const char * command_str) {
        uint8_t len = strlen(command_str);
        uart_write_bytes(UART_NUM_1, command_str, len);
        ESP_LOGI(TAG, "Sent command: %s", command_str);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);
 

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
        {
            .text_font = &font_puhui_16_4,
            .icon_font = &font_awesome_16_4,
    #if CONFIG_USE_WECHAT_MESSAGE_STYLE
            .emoji_font = font_emoji_32_init(),
    #else
            .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
    #endif
        });
    }


 
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

   // MCP工具初始化，添加前进/后退/左转/右转
    void InitializeTools() {
#if CONFIG_IOT_PROTOCOL_MCP
        auto& mcp_server = McpServer::GetInstance();
        // mcp tool has a argument that accepts 0-15
        mcp_server.AddTool("Self.mode.change",
            "Change the robot motion mode, minimum is 1, max is 35, mode 1-4, 16-19 is walking mode, 5-8 is fighting mode, 9-12 is dance mode, 13-15 is record mode.",   
            PropertyList({Property("mode", PropertyType::kPropertyTypeInteger, 0, 35)}),
            [this](const PropertyList& args) {
                int mode = args["mode"].value<int>();
                ESP_LOGI(TAG, "MCP Tool: Change Mode to %d", mode);
                // You can use 'mode' here to change the robot's mode as needed
                this->switchmode(mode);
                return true;
            });
        mcp_server.AddTool("self.motion.forward",
            "Move the robot forward.",
            PropertyList(), [this](const PropertyList&) {
                ESP_LOGI(TAG, "MCP Tool: Forward");
                // TODO: Add hardware control code here
                this->forward();
                return true;
            });
        mcp_server.AddTool("self.motion.backward",
            "Move the robot backward.",
            PropertyList(), [this](const PropertyList&) {
                ESP_LOGI(TAG, "MCP Tool: Backward");
                // TODO: Add hardware control code here
                this->backward();
                return true;
            });
        mcp_server.AddTool("self.motion.turn_left",
            "Turn the robot left.",
            PropertyList(), [this](const PropertyList&) {
                ESP_LOGI(TAG, "MCP Tool: Turn Left");
                // TODO: Add hardware control code here
                this->left();
                return true;
            });
        mcp_server.AddTool("self.motion.turn_right",
            "Turn the robot right.",
            PropertyList(), [this](const PropertyList&) {
                ESP_LOGI(TAG, "MCP Tool: Turn Right");
                // TODO: Add hardware control code here
                this->right();
                return true;
            });
        mcp_server.AddTool("self.motion.stop",
            "Stop the robot.",
            PropertyList(), [this](const PropertyList&) {
                ESP_LOGI(TAG, "MCP Tool: Stop");    
                this->stop();
                return true;
            });

        // 添加一个工具来支持站立动作

        // 添加一个工具来做跳舞动作，支持四种跳舞动作，芭蕾，波浪，手指，dab
        // 该方法有两个参数，舞蹈种类和时间
        // 使用定时器定时发送跳舞动作
        mcp_server.AddTool("self.motion.dance",
            "Make the robot dance.",
            PropertyList({
            Property("dance_type", PropertyType::kPropertyTypeString),
            Property("duration", PropertyType::kPropertyTypeInteger, 5000, 1000, 30000)
            }),
            [this](const PropertyList& args) {
            std::string dance_type = args["dance_type"].value<std::string>();
            int duration = args["duration"].value<int>();
            ESP_LOGI(TAG, "MCP Tool: Dance Type: %s, Duration: %d ms", dance_type.c_str(), duration);
            this->dance(const_cast<char*>(dance_type.c_str()), duration);
            return true;
            });
#endif
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
#elif CONFIG_IOT_PROTOCOL_MCP
        static LampController lamp(LAMP_GPIO);
        // add a robot controller to control the robot through uart2

#endif
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeIot();
        InitializeUart(); // 新增
        SendMatrixIdleFrame(); // 发送默认空闲帧
        InitializeTools(); // 新增
        InitializeSendFrameTimer(); // 新增
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        // switchmode(0);
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
