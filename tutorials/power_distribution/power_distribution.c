#include "libs/gpio/api.h"
#include "libs/gpio/pin_defs.h"

gpio_t DASHBOARD_POWER = PB7;
gpio_t SERVICE_SECTION_POWER = PB1;
gpio_t THROTTLE_POWER = PB2;

char devices[] = {
    DASHBOARD_POWER,
    SERVICE_SECTION_POWER,
    THROTTLE_POWER
};

int state[4] = {0, 0, 0, 0};

void power_device(device) {
    for (int i=0; i<=4; i++) {
        if (devices[i] == device) {
            gpio_set_pin(device, OUTPUT)
        }
    }
}

void power_all(device) {
    for (int i=0; i<=4; i++) {
        gpio_set_pin(device, OUTPUT)
    }
}

void power_off_device(device) {
    for (int i=0; i<=4; i++) {
        if (devices[i] == device) {
            gpio_clear_pin(device, OUTPUT)
        }
    }
}

void power_off_all(device) {
    for (int i=0; i<=4; i++) {
        gpio_clear_pin(device, OUTPUT)
    }
}