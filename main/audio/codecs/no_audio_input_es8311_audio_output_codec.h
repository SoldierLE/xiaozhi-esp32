#ifndef _NO_AUDIO_INPUT_ES8311_AUDIO_OUTPUT_CODEC_H
#define _NO_AUDIO_INPUT_ES8311_AUDIO_OUTPUT_CODEC_H

#include "audio_codec.h"
#include "es8311_audio_codec.h"

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>
#include <mutex>

class NoAudioInputEs8311AudioOutputCodec : public AudioCodec {
private:
    // ES8311相关
    const audio_codec_data_if_t* data_if_ = nullptr;
    const audio_codec_ctrl_if_t* ctrl_if_ = nullptr;
    const audio_codec_if_t* codec_if_ = nullptr;
    const audio_codec_gpio_if_t* gpio_if_ = nullptr;
    esp_codec_dev_handle_t dev_ = nullptr;
    gpio_num_t pa_pin_ = GPIO_NUM_NC;
    bool pa_inverted_ = false;
    
    // I2S通道
    i2s_chan_handle_t tx_handle_ = nullptr;  // ES8311输出
    i2s_chan_handle_t rx_handle_ = nullptr;  // INMP441输入
    
    // 配置参数
    int input_sample_rate_;
    int output_sample_rate_;
    std::mutex data_if_mutex_;

    // INMP441 GPIO配置
    gpio_num_t mic_sck_gpio_;
    gpio_num_t mic_ws_gpio_;
    gpio_num_t mic_din_gpio_;

    void CreateOutputChannel(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout);
    void CreateInputChannel(gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din);
    void UpdateDeviceState();

    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

public:
    NoAudioInputEs8311AudioOutputCodec(void* i2c_master_handle, i2c_port_t i2c_port, 
                                      int input_sample_rate, int output_sample_rate,
                                      gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, 
                                      gpio_num_t dout, gpio_num_t pa_pin, 
                                      uint8_t es8311_addr, bool use_mclk = true, bool pa_inverted = false,
                                      gpio_num_t mic_sck = GPIO_NUM_NC, gpio_num_t mic_ws = GPIO_NUM_NC, gpio_num_t mic_din = GPIO_NUM_NC);
    virtual ~NoAudioInputEs8311AudioOutputCodec();

    virtual void SetOutputVolume(int volume) override;
    virtual void EnableInput(bool enable) override;
    virtual void EnableOutput(bool enable) override;
};

#endif // _NO_AUDIO_INPUT_ES8311_AUDIO_OUTPUT_CODEC_H