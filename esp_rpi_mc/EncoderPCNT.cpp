#include "esp32-hal-gpio.h"
#include "driver/pcnt.h"
#include <stdint.h>
#include "EncoderPCNT.h"

static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
  EncoderPCNT *enc = (EncoderPCNT *)arg;

  // 獲取事件狀態
  uint32_t status;
  pcnt_get_event_status(enc->_pcnt_unit, &status);

  // 處理高限制事件
  if (status & PCNT_EVT_H_LIM)
  {
    enc->_totalCount += H_LIM;
    // Serial.println("High limit event triggered");
  }
  // 處理低限制事件
  else if (status & PCNT_EVT_L_LIM)
  {
    enc->_totalCount += L_LIM;
    // Serial.println("Low limit event triggered");
  }
}

EncoderPCNT::EncoderPCNT(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit)
    : _pinA(pinA), _pinB(pinB), _pcnt_unit(unit), _totalCount(0) {}

EncoderPCNT::~EncoderPCNT()
{
}

void EncoderPCNT::begin()
{
  // 配置 PCNT 單元
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = _pinA;
  pcnt_config.ctrl_gpio_num = _pinB;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = _pcnt_unit;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = H_LIM;
  pcnt_config.counter_l_lim = L_LIM;

  pcnt_unit_config(&pcnt_config);

  pcnt_set_filter_value(_pcnt_unit, 1000); // 設置濾波器值
  pcnt_filter_enable(_pcnt_unit);          // 啟用濾波器

  pcnt_event_enable(_pcnt_unit, PCNT_EVT_H_LIM); // 啟用高限制事件
  pcnt_event_enable(_pcnt_unit, PCNT_EVT_L_LIM); // 啟用低限制事件

  int a = pcnt_isr_service_install(0);                                   // 安裝 ISR 服務
  Serial.printf("errCode: %d\n",a);

  a=pcnt_isr_handler_add(_pcnt_unit, pcnt_intr_handler , (void *)this); // 註冊 ISR 回調函數
  Serial.printf("errCode: %d\n",a);
  
  a=pcnt_intr_enable(_pcnt_unit);
  Serial.printf("errCode: %d\n",a);

  pcnt_counter_pause(_pcnt_unit);  // 暫停計數器
  pcnt_counter_clear(_pcnt_unit);  // 清除計數器
  pcnt_counter_resume(_pcnt_unit); // 恢復計數器
}

int32_t EncoderPCNT::get_count()
{
  int16_t count = 0;
  pcnt_get_counter_value(_pcnt_unit, &count); // 獲取計數器值
  return (int32_t)count + _totalCount;                  // 返回計數器值加上總計數
}

void EncoderPCNT::resetEncoder()
{
  pcnt_counter_pause(_pcnt_unit);  // 暫停計數器
  pcnt_counter_clear(_pcnt_unit);  // 清除計數器
  _totalCount = 0;                 // 重置總計數
  pcnt_counter_resume(_pcnt_unit); // 恢復計數器
}
