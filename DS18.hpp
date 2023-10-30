#ifndef DS18B20_HPP  // 更改宏的名称为DS18B20_HPP
#define DS18B20_HPP

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

class DS18B20 {
public:
    DS18B20(GPIO_TypeDef* gpio_port, uint16_t gpio_pin);
    void init();
    float readTemperature();
    float temperature;
    
private:
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;

    
    void writeBit(uint8_t bit);
    uint8_t readBit();
    void writeByte(uint8_t data);
    uint8_t readByte();
};

#endif
