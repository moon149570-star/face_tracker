#include "key.h"

static GPIO_TypeDef* key_ports[] = {GPIOA, GPIOA, GPIOB, GPIOB};
static uint16_t key_pins[] = {GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_1, GPIO_PIN_11};

volatile uint8_t key_states[4] = {0, 0, 0, 0}; // Store the state of each key (pressed or not)
static uint8_t prev_states[4];
static uint8_t current_states[4];
void Key_Init(void)
{
    for(int index = 0; index < 4; index++)
    {
        prev_states[index] = 0;
        current_states[index] = 0;
    }
}

void Key_Scan(void)
{

    for(int index = 0; index < 4; index++)
    {
        current_states[index] = HAL_GPIO_ReadPin(key_ports[index], key_pins[index]) == GPIO_PIN_RESET ? 1 : 0;
        if(current_states[index] == 1 && prev_states[index] == 0)
        {
            key_states[index] = 1; // Key pressed
        }
        else if(current_states[index] == 0 && prev_states[index] == 1)
        {
            key_states[index] = 0; // Key released
        }
        prev_states[index] = current_states[index]; // Update previous state
    }
}
