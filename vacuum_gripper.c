#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "XGZP6847D.h"

// L298N Motor Driver Configuration for Vacuum Pump
#define DIGITAL_INPUT0_PIN 2      // GPIO2: Control signal (LOW -> HIGH starts pump)
#define DIGITAL_OUTPUT0_PIN 5     // GPIO3: Output signal (HIGH when target pressure reached)
#define L298_IN1_PIN       28     // GPIO28: L298N IN1 pin
#define L298_IN2_PIN       29     // GPIO29: L298N IN2 pin
#define L298_ENABLE_A_PIN  15     // GPIO15: L298N Enable_A pin (ENA)

// L298N Motor Driver Configuration for Vent Valve
#define L298_IN3_PIN       11     // GPIO11: L298N IN3 pin (vent valve)
#define L298_IN4_PIN       12     // GPIO12: L298N IN4 pin (vent valve)
#define L298_ENABLE_B_PIN  14     // GPIO14: L298N Enable_B pin (ENB)
// Target pressure level: -60kPa = -57764 Pa /according to measurements/
// #define TARGET_PRESSURE_PA -57764.0f
#define TARGET_PRESSURE_PA -70000.0f
// ============================================================================
// Vacuum Pump Control Functions (L298N Motor Driver)
// ============================================================================

/**
 * Disable vacuum pump by turning off Enable_A and setting IN1/IN2 to LOW
 */
static void disable_vacuum_pump(void)
{
    gpio_put(L298_ENABLE_A_PIN, 0);
    gpio_put(L298_IN1_PIN, 0);
    gpio_put(L298_IN2_PIN, 0);
    printf("Vacuum pump disabled\n");
}

/**
 * Enable vacuum pump by setting IN1=HIGH, IN2=LOW, and Enable_A=HIGH
 */
static void enable_vacuum_pump(void)
{
    gpio_put(L298_IN1_PIN, 1);
    gpio_put(L298_IN2_PIN, 0);
    gpio_put(L298_ENABLE_A_PIN, 1);
    printf("Vacuum pump enabled\n");
}

/**
 * Enable vent valve by setting IN3=HIGH, IN4=LOW
 */
static void enable_vent_valve(void)
{
    gpio_put(L298_IN4_PIN, 1);
    gpio_put(L298_IN3_PIN, 0);
    gpio_put(L298_ENABLE_B_PIN, 1);
    printf("Vent valve enabled\n");
}

/**
 * Disable vent valve by setting IN3=LOW, IN4=LOW
 */
static void disable_vent_valve(void)
{
    gpio_put(L298_IN3_PIN, 0);
    gpio_put(L298_IN4_PIN, 0);
    gpio_put(L298_ENABLE_B_PIN, 0);
    printf("Vent valve disabled\n");
}

/**
 * Initialize GPIO pins for vacuum pump control via L298N motor driver
 */
static void init_vacuum_control(void)
{
    // Configure digital input pin (GPIO2) for control signal
    gpio_init(DIGITAL_INPUT0_PIN);
    gpio_set_dir(DIGITAL_INPUT0_PIN, GPIO_IN);
    gpio_pull_down(DIGITAL_INPUT0_PIN);

    // Configure digital output pin (GPIO3) for pressure status
    gpio_init(DIGITAL_OUTPUT0_PIN);
    gpio_set_dir(DIGITAL_OUTPUT0_PIN, GPIO_OUT);
    gpio_put(DIGITAL_OUTPUT0_PIN, 0);  // Start with LOW

    // Configure L298N control pins for vacuum pump as outputs
    gpio_init(L298_IN1_PIN);
    gpio_set_dir(L298_IN1_PIN, GPIO_OUT);

    gpio_init(L298_IN2_PIN);
    gpio_set_dir(L298_IN2_PIN, GPIO_OUT);

    gpio_init(L298_ENABLE_A_PIN);
    gpio_set_dir(L298_ENABLE_A_PIN, GPIO_OUT);

    // Configure L298N control pins for vent valve as outputs
    gpio_init(L298_IN3_PIN);
    gpio_set_dir(L298_IN3_PIN, GPIO_OUT);

    gpio_init(L298_IN4_PIN);
    gpio_set_dir(L298_IN4_PIN, GPIO_OUT);

    gpio_init(L298_ENABLE_B_PIN);
    gpio_set_dir(L298_ENABLE_B_PIN, GPIO_OUT);

    // Start with pump and vent valve disabled
    disable_vacuum_pump();
    disable_vent_valve();
}

int main()
{
    stdio_init_all();
    
    // Wait for USB UART to be ready
    sleep_ms(2000);
    
    printf("Initializing XGZP6847D Pressure Sensor...\n");
    printf("I2C Address: 0x%02X\n", PRESSURE_SENSOR_ADDR);
    printf("SDA: GPIO%d, SCL: GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize hardware
    init_i2c_pressure_sensor();
    init_vacuum_control();
    
    printf("I2C initialized. Starting pressure monitoring...\n\n");

    // Initialize state tracking for digital input
    bool prev_input_state = gpio_get(DIGITAL_INPUT0_PIN);
    if (prev_input_state) {
        enable_vacuum_pump();
    }

    while (true) {
        // Measure pressure first
        if (!trigger_measurement(50)) {
            printf("Measurement trigger failed\n");
            sleep_ms(100);
            continue;
        }

        int32_t raw_pressure = read_pressure_raw();
        float pressure_pa = convert_to_pascal(raw_pressure);

        // Control digital_output_0 based on target pressure (-65kPa)
        if (fabs(pressure_pa) >= fabs(TARGET_PRESSURE_PA)) {
            // Pressure reached or exceeded target level
            if (!gpio_get(DIGITAL_OUTPUT0_PIN)) {
                gpio_put(DIGITAL_OUTPUT0_PIN, 1);
                printf("Target pressure reached: %.2f Pa. digital_output_0 set to HIGH\n", pressure_pa);

            }
        } else {
            // Pressure dropped below target level
            if (gpio_get(DIGITAL_OUTPUT0_PIN)) {
                gpio_put(DIGITAL_OUTPUT0_PIN, 0);
                printf("Pressure below target: %.2f Pa. digital_output_0 reset to LOW\n", pressure_pa);

            }
        }

        // Monitor digital input for vacuum pump control
        bool input_state = gpio_get(DIGITAL_INPUT0_PIN);

        // Turn on pump when signal goes LOW to HIGH (rising edge)
        if (input_state && !prev_input_state) {
            printf("digital_input_0 rising edge detected\n");
            enable_vacuum_pump();
        }
        // Turn off pump and activate vent valve when signal goes HIGH to LOW (falling edge)
        else if (!input_state && prev_input_state) {
            printf("digital_input_0 falling edge detected\n");
            disable_vacuum_pump();
            
            // Turn on vent valve for 1 second
            enable_vent_valve();
            sleep_ms(1000);
            disable_vent_valve();
        }

        prev_input_state = input_state;

        // Display pressure data and status
        bool output0_status = gpio_get(DIGITAL_OUTPUT0_PIN);
        printf("Pressure - Raw: %d (0x%06X), Pa: %.2f, Target: %.2f Pa, digital_output_0: %s\n",
                (int)raw_pressure, (unsigned int)(raw_pressure & 0xFFFFFF), pressure_pa, TARGET_PRESSURE_PA,
                output0_status ? "HIGH" : "LOW");
        sleep_ms(100);  // Read every 100ms for better responsiveness
    }
}

