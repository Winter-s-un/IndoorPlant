#include "DS18.hpp"

DS18B20::DS18B20(GPIO_TypeDef* gpio_port, uint16_t gpio_pin)
    : gpio_port(gpio_port), gpio_pin(gpio_pin) {
    init();
}

void DS18B20::init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpio_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}

void DS18B20::writeBit(uint8_t bit) {
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
    if (bit) {
        vTaskDelay(1);  // Delay for required time (you may need to adjust this)
        HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
    } else {
        vTaskDelay(1);  // Delay for required time (you may need to adjust this)
        HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
    }
}

uint8_t DS18B20::readBit() {
    uint8_t bit = 0;
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
    vTaskDelay(1);  // Delay for required time (you may need to adjust this)
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
    vTaskDelay(1);  // Delay for required time (you may need to adjust this);
    if (HAL_GPIO_ReadPin(gpio_port, gpio_pin)) {
        bit = 1;
    }
    vTaskDelay(50);  // Delay to complete the bit read (you may need to adjust this)
    return bit;
}

void DS18B20::writeByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        writeBit((data >> i) & 0x01);
    }
}

uint8_t DS18B20::readByte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (readBit() << i);
    }
    return byte;
}

static volatile int indicate = 0;
static volatile int Byte = 0;
float DS18B20::readTemperature() {
    // Send command to start temperature conversion
    writeByte(0x44);  // 0x44 is the command to start a single temperature conversion
    indicate = 1;
    // Wait for conversion to complete (750ms for 12-bit resolution)
    vTaskDelay(750);
    
    // Reset the 1-Wire bus
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
    vTaskDelay(480);
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
    vTaskDelay(40);
    indicate = 2;
    // Read temperature data
    writeByte(0xBE);  // Read Scratchpad command
    static uint16_t tempData = (readByte() | (readByte() << 8));
    Byte = readByte();
    indicate = 3;
    // Calculate temperature in degrees Celsius
    temperature = (float)tempData / 16.0;

    return temperature;
}
