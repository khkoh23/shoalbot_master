#include "estop.h"
ESTOP::ESTOP() {
    ESTOP0_timer.speed_mode = PWM_MODE;
    ESTOP0_timer.duty_resolution = PWM_DUTY_RES;
    ESTOP0_timer.timer_num = LEDC_TIMER_0;
    ESTOP0_timer.freq_hz = PWM_FREQUENCY;  // Set output frequency at 4 kHz
    ESTOP0_timer.clk_cfg = LEDC_AUTO_CLK;

    ESTOP0_channel.speed_mode = PWM_MODE;
    ESTOP0_channel.channel = PWM_CHANNEL_0;
    ESTOP0_channel.timer_sel = LEDC_TIMER_0;
    ESTOP0_channel.intr_type = LEDC_INTR_DISABLE;
    ESTOP0_channel.gpio_num = PWM_OUTPUT_IO;
    ESTOP0_channel.duty = pwm_duty; // Set duty to 0%
    ESTOP0_channel.hpoint = 0;

    ESTOP1_timer.speed_mode = PWM_MODE;
    ESTOP1_timer.duty_resolution = PWM_DUTY_RES;
    ESTOP1_timer.timer_num = LEDC_TIMER_1;
    ESTOP1_timer.freq_hz = PWM_FREQUENCY;  // Set output frequency at 4 kHz
    ESTOP1_timer.clk_cfg = LEDC_AUTO_CLK;

    ESTOP1_channel.gpio_num = PWM_OUTPUT_IO1;
    ESTOP1_channel.speed_mode = PWM_MODE;
    ESTOP1_channel.channel = PWM_CHANNEL_1; 
    ESTOP1_channel.intr_type = LEDC_INTR_DISABLE;
    ESTOP1_channel.timer_sel = LEDC_TIMER_1;
    ESTOP1_channel.duty = pwm_duty;
    ESTOP1_channel.hpoint = 0;
}

void ESTOP::begin() {
    ESP_ERROR_CHECK(ledc_timer_config(&ESTOP0_timer));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(ledc_timer_config(&ESTOP1_timer));

    ESP_ERROR_CHECK(ledc_channel_config(&ESTOP0_channel));
    ESP_ERROR_CHECK(ledc_channel_config(&ESTOP1_channel));
}

void ESTOP::set_duty_cycle(double duty_cycle) {
    pwm_duty = pow(2, 13) * duty_cycle;
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_0, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_1));
}

void ESTOP::stop_pulse() {
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_1));
}