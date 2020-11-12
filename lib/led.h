#pragma once

#include "stdint.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

class Led {
    public:
        Led();

        void init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
        void blink(uint32_t onTime, uint32_t period);
        void strobe(uint32_t onOffTime, uint16_t repeats);
        void ledOn();
        void ledOff();
        void update(uint32_t tick);

    private:
        void start();

        uint8_t ledPin;
        GPIO_TypeDef* ledPort;
        uint32_t ledOnTime;
        uint32_t ledOffTime;
        uint32_t periodTime;
        uint32_t startTime;
        uint32_t strobeStartTime;
        uint16_t strobeRepeats;
        uint32_t strobeOnOffTime;
        bool manualDrive;
};
