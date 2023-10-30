#ifndef SHT20_HPP
#define SHT20_HPP

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "main.h"


class SHT20 {
public:
    SHT20(I2C_HandleTypeDef* i2c2Handle);
    bool init();
    bool readTemperatureHumidity(float& temperature, float& humidity);

private:
    I2C_HandleTypeDef* i2c2Handle;
    uint8_t readData(uint8_t command, uint8_t* data, uint8_t dataSize);
};

#endif // SHT20_HPP
