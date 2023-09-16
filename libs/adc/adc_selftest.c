#include "libs/adc/api.h"
#include <stdlib.h>

typedef struct adc_selftest {
    uint16_t adc_values[ADC_COUNT],
} adc_data;

adc_data curr_adc_data;

void adc_init(avoid) {}

void adc_start_convert(adc_pin_e pin) {}

int adc_poll_complete(uint16_t* result) {
    return 0;
}

void adc_interrupt_enable(void (*callback)(void)) {}

uint16_t adc_read(adc_pin_e pin) {
    return curr_adc_data.adc_values[pin];
}

void set_adc_val(adc_pin_e pin, uint16_t test_val) {
    curr_adc_data.adc_values[pin] = test_val;
}