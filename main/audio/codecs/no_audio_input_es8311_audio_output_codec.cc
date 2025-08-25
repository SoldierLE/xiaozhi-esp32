#include "no_audio_input_es8311_audio_output_codec.h"

#include <esp_log.h>
#include <cmath>

#define TAG "NoInputEs8311Output"

NoAudioInputEs8311AudioOutputCodec::NoAudioInputEs8311AudioOutputCodec(void* i2c_master_handle, i2c_port_t i2c_port, 
                                                                       int input_sample_rate, int output_sample_rate,
                                                                       gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, 
                                                                       gpio_num_t dout, gpio_num_t pa_pin, 
                                                                       uint8_t es8311_addr, bool use_mclk, bool pa_inverted,
                                                                       gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
    duplex_ = false; // 支持双工：INMP441输入 + ES8311输出
    input_reference_ = false;
    input_channels_ = 1; // 有输入通道
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;
    pa_pin_ = pa_pin;
    pa_inverted_ = pa_inverted;
    
    // 保存INMP441 GPIO配置
    mic_sck_gpio_ = mic_sck;
    mic_ws_gpio_ = mic_ws;
    mic_din_gpio_ = mic_din;

    assert(input_sample_rate_ == output_sample_rate_);
    // 创建输出通道（ES8311）
    CreateOutputChannel(mclk, bclk, ws, dout);
    
    // 创建输入通道（INMP441）
    if (mic_sck != GPIO_NUM_NC && mic_ws != GPIO_NUM_NC && mic_din != GPIO_NUM_NC) {
        CreateInputChannel(mic_sck, mic_ws, mic_din);
    }

    // 初始化ES8311接口
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = rx_handle_, // INMP441输入
        .tx_handle = tx_handle_, // ES8311输出
    };
    data_if_ = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if_ != NULL);

    // 输出控制接口
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = i2c_port,
        .addr = es8311_addr,
        .bus_handle = i2c_master_handle,
    };
    ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if_ != NULL);

    gpio_if_ = audio_codec_new_gpio();
    assert(gpio_if_ != NULL);

    // ES8311配置
    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.ctrl_if = ctrl_if_;
    es8311_cfg.gpio_if = gpio_if_;
    es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC; // 支持输出
    es8311_cfg.pa_pin = pa_pin;
    es8311_cfg.use_mclk = use_mclk;
    es8311_cfg.hw_gain.pa_voltage = 5.0;
    es8311_cfg.hw_gain.codec_dac_voltage = 3.3;
    es8311_cfg.pa_reverted = pa_inverted_;
    codec_if_ = es8311_codec_new(&es8311_cfg);
    assert(codec_if_ != NULL);
    
    ESP_LOGI(TAG, "NoAudioInputEs8311AudioOutputCodec initialized with INMP441 input");
}

NoAudioInputEs8311AudioOutputCodec::~NoAudioInputEs8311AudioOutputCodec() {
    if (dev_ != nullptr) {
        esp_codec_dev_delete(dev_);
    }

    if (codec_if_ != nullptr) {
        audio_codec_delete_codec_if(codec_if_);
    }
    if (ctrl_if_ != nullptr) {
        audio_codec_delete_ctrl_if(ctrl_if_);
    }
    if (gpio_if_ != nullptr) {
        audio_codec_delete_gpio_if(gpio_if_);
    }
    if (data_if_ != nullptr) {
        audio_codec_delete_data_if(data_if_);
    }
    
    // if (tx_handle_ != nullptr) {
    //     ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    //     ESP_ERROR_CHECK(i2s_del_channel(tx_handle_));
    // }
    
    if (rx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
        ESP_ERROR_CHECK(i2s_del_channel(rx_handle_));
    }
}

void NoAudioInputEs8311AudioOutputCodec::CreateOutputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout) {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            #ifdef I2S_HW_VERSION_2    
                .ext_clk_freq_hz = 0,
            #endif
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef I2S_HW_VERSION_2   
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED, // 输出通道不需要输入
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_LOGI(TAG, "ES8311 output channel created");
}

void NoAudioInputEs8311AudioOutputCodec::CreateInputChannel(gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1, // 使用相同的I2S端口
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)input_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            #ifdef I2S_HW_VERSION_2    
                .ext_clk_freq_hz = 0,
            #endif
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef I2S_HW_VERSION_2   
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // INMP441不需要MCLK
            .bclk = mic_sck,
            .ws = mic_ws,
            .dout = I2S_GPIO_UNUSED, // 输入通道不需要输出
            .din = mic_din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "INMP441 input channel created");
}

void NoAudioInputEs8311AudioOutputCodec::UpdateDeviceState() {
    if ((input_enabled_ || output_enabled_) && dev_ == nullptr) {
        esp_codec_dev_cfg_t dev_cfg = {
            .dev_type = ESP_CODEC_DEV_TYPE_OUT, // 支持输入和输出
            .codec_if = codec_if_,
            .data_if = data_if_,
        };
        dev_ = esp_codec_dev_new(&dev_cfg);
        assert(dev_ != NULL);

        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = 1,
            .channel_mask = 0,
            .sample_rate = (uint32_t)input_sample_rate_,
            .mclk_multiple = 0,
        };
        ESP_ERROR_CHECK(esp_codec_dev_open(dev_, &fs));
        ESP_ERROR_CHECK(esp_codec_dev_set_in_gain(dev_, AUDIO_CODEC_DEFAULT_MIC_GAIN));
        ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(dev_, output_volume_));
    } else if (!input_enabled_ && !output_enabled_ && dev_ != nullptr) {
        esp_codec_dev_close(dev_);
        dev_ = nullptr;
    }
    
    if (pa_pin_ != GPIO_NUM_NC) {
        int level = output_enabled_ ? 1 : 0;
        gpio_set_level(pa_pin_, pa_inverted_ ? !level : level);
    }
}

void NoAudioInputEs8311AudioOutputCodec::SetOutputVolume(int volume) {
    if (dev_ != nullptr) {
        ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(dev_, volume));
    }
    AudioCodec::SetOutputVolume(volume);
}

void NoAudioInputEs8311AudioOutputCodec::EnableInput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == input_enabled_) {
        return;
    }
    AudioCodec::EnableInput(enable);
    UpdateDeviceState();
}

void NoAudioInputEs8311AudioOutputCodec::EnableOutput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == output_enabled_) {
        return;
    }
    AudioCodec::EnableOutput(enable);
    UpdateDeviceState();
}

int NoAudioInputEs8311AudioOutputCodec::Read(int16_t* dest, int samples) {
    if (input_enabled_ && rx_handle_ != nullptr) {
        size_t bytes_read;
        std::vector<int32_t> bit32_buffer(samples);
        
        if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "INMP441 read failed!");
            return 0;
        }

        samples = bytes_read / sizeof(int32_t);
        for (int i = 0; i < samples; i++) {
            int32_t value = bit32_buffer[i] >> 12;
            dest[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
        }
        return samples;
    }
    return 0;
}

int NoAudioInputEs8311AudioOutputCodec::Write(const int16_t* data, int samples) {
    if (output_enabled_ && dev_ != nullptr) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_codec_dev_write(dev_, (void*)data, samples * sizeof(int16_t)));
        return samples;
    }
    return 0;
}