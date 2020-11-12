#include "led.h"

Led::Led() : ledPin(255), ledPort(nullptr), ledOnTime(0), ledOffTime(0),
             periodTime(0), startTime(0),
             strobeStartTime(0), strobeRepeats(0), strobeOnOffTime(0), manualDrive(nullptr) {}

void Led::init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    ledPort = GPIOx;
    ledPin = GPIO_Pin;
}

void Led::blink(uint32_t onTime, uint32_t period)
{
    startTime = HAL_GetTick();
    ledOnTime = onTime;
    ledOffTime = period - onTime;
    periodTime = period;
    start();
}

void Led::strobe(uint32_t onOffTime, uint16_t repeats)
{
    strobeStartTime = HAL_GetTick();
    strobeRepeats = 2 * repeats;
    strobeOnOffTime = onOffTime;
    start();
}

void Led::ledOn()
{
    HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);
    manualDrive = true;
    strobeRepeats = 0;
}

void Led::ledOff()
{
    HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_RESET);
    manualDrive = true;
}

void Led::start()
{
    // if initialization proceeded
    if (ledPin != 255 || ledPort != nullptr) {
        
        HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);
        manualDrive = false;
    }
}

void Led::update(uint32_t tick)
{
    if (manualDrive == false) {
        if (strobeRepeats > 0) {
            if (tick > strobeStartTime + strobeOnOffTime) {
                strobeRepeats--;
                strobeStartTime = tick;
                HAL_GPIO_TogglePin(ledPort, ledPin);
            }
        }
        else {
            if (tick > startTime + ledOnTime && tick < startTime + periodTime) {
                HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_RESET);
            }
            else if (tick > startTime + periodTime) {
                HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);
                startTime = HAL_GetTick();
            }
        }
    }
}