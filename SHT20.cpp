#include "SHT20.hpp"

#define Delay_TIMEOUT pdMS_TO_TICKS(100)
uint8_t SHT20_I2C_ADDRESS = 0x40;


SHT20::SHT20(I2C_HandleTypeDef* i2c2Handle) : i2c2Handle(i2c2Handle) {}

bool SHT20::init() {
    // 发送初始化命令，这里需要根据SHT20的数据手册进行设置
    uint8_t initCommand = 0x80; // 0x20 是一个示例，具体命令根据数据手册设置
    if (HAL_I2C_Master_Transmit(i2c2Handle, SHT20_I2C_ADDRESS, &initCommand, 1, HAL_MAX_DELAY) == HAL_OK) {
    // 初始化成功
    HAL_Delay(100); // 延时是示例，根据传感器要求进行调整
    return true;
} else {
    // 初始化失败，记录错误信息
    uint32_t error = HAL_I2C_GetError(i2c2Handle);
    // 在这里可以使用调试信息记录错误代码
    // 可以使用error变量来查看错误代码
    return false;
}

}

bool SHT20::readTemperatureHumidity(float& temperature, float& humidity) {
    uint8_t command = 0xE3; // 读取温度和湿度的命令
    uint8_t data[3];
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;

    // 发送启动命令
    if (HAL_I2C_Master_Transmit(i2c2Handle, SHT20_I2C_ADDRESS, &command, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false; // 发送命令失败
    }

    // 延时，等待数据准备好
     HAL_Delay(100); // 50 ms 延时是一个示例，请根据传感器规格调整

    // 读取数据
    if (HAL_I2C_Master_Receive(i2c2Handle, SHT20_I2C_ADDRESS, data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return false; // 读取数据失败
    }

    // 解析温度和湿度数据
    uint16_t rawTemperature = ((uint16_t)data[0] << 8) | data[1];
    temperature = -46.85 + (175.72 * rawTemperature / 65536.0);

    uint16_t rawHumidity = ((uint16_t)data[2] << 8) | data[3];
    humidity = -6.0 + (125.0 * rawHumidity / 65536.0);


    return true;
}

