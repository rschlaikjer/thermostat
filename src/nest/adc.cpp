#include <nest/adc.hpp>

uint8_t channel_array[] = { 8 };

void adc_setup(void) {
    // Enable clock for ADC and GPIOB
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Set our pin (PB0) as analog in
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

    // Turn off the ADC
    adc_power_off(NEST_ADC);

    // Set the clock source
    adc_set_clk_source(NEST_ADC, ADC_CLKSOURCE_ADC);

    // Calibrate
    adc_calibrate(NEST_ADC);

    // Using scan mode, though doesn't really matter since
    // we're only using one source
    adc_set_operation_mode(NEST_ADC, ADC_MODE_SCAN);

    // No ext trigger
    adc_disable_external_trigger_regular(NEST_ADC);

    // Alighn right
    adc_set_right_aligned(NEST_ADC);

    // We have our own temp sensors, thanks
    adc_disable_temperature_sensor();

    adc_set_sample_time_on_all_channels(NEST_ADC, ADC_SMPTIME_071DOT5);
    adc_set_regular_sequence(NEST_ADC, 1, channel_array);
    adc_set_resolution(NEST_ADC, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(NEST_ADC);
    adc_power_on(NEST_ADC);
}

uint16_t adc_read() {
    adc_start_conversion_regular(NEST_ADC);
    while (!(adc_eoc(NEST_ADC)));
    return adc_read_regular(NEST_ADC);
}
