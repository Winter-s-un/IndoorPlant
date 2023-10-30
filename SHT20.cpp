#include "SHT20.hpp"

#define Delay_TIMEOUT pdMS_TO_TICKS(100)
uint8_t SHT20_I2C_ADDRESS = 0x40;


SHT20::SHT20(I2C_HandleTypeDef* i2c2Handle) : i2c2Handle(i2c2Handle) {}

bool SHT20::init() {

    uint8_t initCommand = 0x80; 
    if (HAL_I2C_Master_Transmit(i2c2Handle, SHT20_I2C_ADDRESS, &initCommand, 1, HAL_MAX_DELAY) == HAL_OK) {

    HAL_Delay(100); 
    return true;
} else {

    uint32_t error = HAL_I2C_GetError(i2c2Handle);

    return false;
}

}

bool SHT20::readTemperatureHumidity(float& temperature, float& humidity) {
    uint8_t command = 0xE3;
    uint8_t data[3];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;


    if (HAL_I2C_Master_Transmit(i2c2Handle, SHT20_I2C_ADDRESS, &command, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false; 
    }


     HAL_Delay(100); 


    if (HAL_I2C_Master_Receive(i2c2Handle, SHT20_I2C_ADDRESS, data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return false; 
    }


    uint16_t rawTemperature = ((uint16_t)data[0] << 8) | data[1];
    temperature = -46.85 + (175.72 * rawTemperature / 65536.0);

    uint16_t rawHumidity = ((uint16_t)data[2] << 8) | data[3];
    humidity = -6.0 + (125.0 * rawHumidity / 65536.0);


    return true;
}

