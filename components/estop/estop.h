#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "math.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PWM1 GPIO_NUM_2
#define PWM2 GPIO_NUM_1

#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_OUTPUT_IO (PWM1) 
#define PWM_OUTPUT_IO1 (PWM2) 
#define PWM_CHANNEL_0 LEDC_CHANNEL_0
#define PWM_CHANNEL_1 LEDC_CHANNEL_1
#define PWM_DUTY_RES LEDC_TIMER_13_BIT // set duty resolution to 13 bits
#define PWM_FREQUENCY (10)

class ESTOP {
    public:
        ESTOP();
        void begin();
        void set_duty_cycle(double duty_cycle); // input in percentage (0~1)
        void stop_pulse();

    private:
        ledc_timer_config_t ESTOP0_timer = {};
        ledc_channel_config_t ESTOP0_channel = {};

        ledc_timer_config_t ESTOP1_timer = {}; 
        ledc_channel_config_t ESTOP1_channel = {};

        uint32_t pwm_duty = 819; // default duty cycle is 10%
};