#include "key.h"
#include "display.h"
#include "main.h"
#include "UART.h"

static char key = '\0';
static uint32_t time = 0;
static uint8_t onClick = 0;

void endClick() {
    time = HAL_GetTick();
    onClick = 0;
}

void action() {
    if (onClick || (HAL_GetTick() - time) < 50) { return; }
    onClick = 1;

    if (key == 'M') {
        onDisplay[0] = onDisplay[0] == 0 ? 1 : 0;
        if (onDisplay[0] != 0) {
            onDisplay[1] = 0;
        }
        display();
        return;
    }

    float* values[2] = {&Flashback, &Settime};

    if (onDisplay[1] == 0) {
        if (key == 'S') {
            onDisplay[1] = onDisplay[0];
            SP = *values[onDisplay[1] - 1];
            display();
            return;
        }
        if (key == 'S') {
            onDisplay[0]--;
        }
        if (onDisplay[0] > 2) {
            onDisplay[0] = 1;
        }
//        if (onDisplay[0] < 1) {
//            onDisplay[0] = 2;
//        }
    } else {
//        if (offReceive == 1) { return; }
        if (key == 'S') {
            onDisplay[0] = onDisplay[0] == 0 ? 1 : 0;
            if (onDisplay[0] == 0) {
                *values[onDisplay[1] - 1] = SP;
                onDisplay[1] = 0;
            }
//            offReceive = 1;

            display();
            return;
        }
        if (key == 'S') {
            SP++;
        }
        if (key == 'S') {
            SP--;
        }
    }
    display();
}

//void enterKey() {
//    uint16_t pin[4] = {GPIO_PIN_14, GPIO_PIN_15};
//    char ch[2] = {'M', 'S'};

//    for (uint8_t i = 0; i < 2; i++) {
//        if (HAL_GPIO_ReadPin(GPIOB, pin[i]) == 1) {
//            if (key != ch[i]) {
//                endClick();
//            }
//            key = ch[i];
//            action();
//            return;
//        }
//    }

//    key = '\0';
//    endClick();
//}