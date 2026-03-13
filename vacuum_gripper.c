#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "XGZP6847D.h"

// L298N Motor Driver Configuration for Vacuum Pump
#define DIGITAL_INPUT0_PIN 2      // GPIO2: DI0 - Direct control for pump (HIGH=ON, LOW=OFF)
#define DIGITAL_INPUT1_PIN 3      // GPIO3: DI1 - Direct control for vent valve (HIGH=ON, LOW=OFF)
#define DIGITAL_OUTPUT0_PIN 5     // GPIO5: Alarm signal (HIGH when pressure > -60000 Pa)
#define DIGITAL_OUTPUT1_PIN 6     // GPIO6: Alarm signal (HIGH when pressure > -60000 Pa)
#define L298_IN1_PIN       28     // GPIO28: L298N IN1 pin
#define L298_IN2_PIN       29     // GPIO29: L298N IN2 pin
#define L298_ENABLE_A_PIN  15     // GPIO15: L298N Enable_A pin (ENA)

// L298N Motor Driver Configuration for Vent Valve
#define L298_IN3_PIN       11     // GPIO11: L298N IN3 pin (vent valve)
#define L298_IN4_PIN       12     // GPIO12: L298N IN4 pin (vent valve)
#define L298_ENABLE_B_PIN  14     // GPIO14: L298N Enable_B pin (ENB)
// Alarm threshold: DO0 turns ON when pressure rises above this value
#define ALARM_PRESSURE_THRESHOLD_PA -60000.0f  // Alarm when pressure > -60000 Pa
// ============================================================================
// Vacuum Pump Control Functions (L298N Motor Driver)
// ============================================================================
#define RB_ROBOT 1

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
}

/**
 * Initialize GPIO pins for vacuum pump control via L298N motor driver
 */
static void init_vacuum_control(void)
{
    // Configure digital input pin DI0 (GPIO2) - Direct control for pump
    gpio_init(DIGITAL_INPUT0_PIN);
    gpio_set_dir(DIGITAL_INPUT0_PIN, GPIO_IN);
    gpio_pull_down(DIGITAL_INPUT0_PIN);

    // Configure digital input pin DI1 (GPIO3) - Direct control for vent valve
    gpio_init(DIGITAL_INPUT1_PIN);
    gpio_set_dir(DIGITAL_INPUT1_PIN, GPIO_IN);
    gpio_pull_down(DIGITAL_INPUT1_PIN);

    // Configure digital output pin (GPIO5) for alarm signal
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
    printf("DI0 (GPIO%d): Direct control for pump (HIGH=ON, LOW=OFF)\n", DIGITAL_INPUT0_PIN);
    printf("DI1 (GPIO%d): Direct control for vent valve (HIGH=ON, LOW=OFF)\n", DIGITAL_INPUT1_PIN);
    printf("DO0 (GPIO%d): Alarm signal (ON when pump running AND pressure > %.2f Pa)\n", DIGITAL_OUTPUT0_PIN, ALARM_PRESSURE_THRESHOLD_PA);
    printf("Control Logic:\n");
    printf("  - Pump: Controlled directly by DI0\n");
    printf("  - Vent Valve: Controlled directly by DI1\n");
    printf("  - DO0 Alarm: Evaluated only when pump ON; ON when pressure > %.2f Pa (vacuum lost); OFF when pump OFF\n\n", ALARM_PRESSURE_THRESHOLD_PA);

    while (true) {
        // Measure pressure first
        if (!trigger_measurement(50)) {
            printf("Measurement trigger failed\n");
            sleep_ms(100);
            continue;
        }

        int32_t raw_pressure = read_pressure_raw();
        float pressure_pa = convert_to_pascal(raw_pressure);

        // Read digital input states
        bool di0_state = false;
        bool di1_state = false;
        if (RB_ROBOT) {
            di0_state = gpio_get(DIGITAL_INPUT0_PIN);  // Direct control for pump
            di1_state = gpio_get(DIGITAL_INPUT1_PIN);  // Direct control for vent valve
        } else {
            di0_state = gpio_get(DIGITAL_INPUT0_PIN);  // Direct control for pump
            di1_state = gpio_get(DIGITAL_INPUT1_PIN);  // Direct control for vent valve

            di0_state = !di0_state;  // Invert the state for the robot
            di1_state = !di1_state;  // Invert the state for the robot
        }

        // Direct control: Pump controlled by DI0
        if (di0_state) {
            // DI0 is HIGH: turn ON pump
            if (!gpio_get(L298_ENABLE_A_PIN)) {
                enable_vacuum_pump();
            }
        } else {
            // DI0 is LOW: turn OFF pump
            if (gpio_get(L298_ENABLE_A_PIN)) {
                disable_vacuum_pump();
            }
        }

        // Direct control: Vent valve controlled by DI1
        if (di1_state) {
            // DI1 is HIGH: turn ON vent valve
            if (!gpio_get(L298_ENABLE_B_PIN)) {
                enable_vent_valve();
            }
        } else {
            // DI1 is LOW: turn OFF vent valve
            if (gpio_get(L298_ENABLE_B_PIN)) {
                disable_vent_valve();
            }
        }

        // DO0 alarm: evaluated ONLY when pump is running. If pump OFF, DO0 stays inactive (LOW).
        // Alarm condition (PNP): pump ON AND pressure above threshold = vacuum lost -> DO0 HIGH.
        bool pump_running = gpio_get(L298_ENABLE_A_PIN);
        if (!pump_running) {
            gpio_put(DIGITAL_OUTPUT0_PIN, 0);
        } else {
            if (pressure_pa > ALARM_PRESSURE_THRESHOLD_PA) {
                if (!gpio_get(DIGITAL_OUTPUT0_PIN)) {
                    gpio_put(DIGITAL_OUTPUT0_PIN, 1);
                    printf("ALARM: Pressure %.2f Pa > %.2f Pa. DO0 set to HIGH\n", pressure_pa, ALARM_PRESSURE_THRESHOLD_PA);
                }
            } else {
                if (gpio_get(DIGITAL_OUTPUT0_PIN)) {
                    gpio_put(DIGITAL_OUTPUT0_PIN, 0);
                    printf("Pressure %.2f Pa <= %.2f Pa. DO0 reset to LOW\n", pressure_pa, ALARM_PRESSURE_THRESHOLD_PA);
                }
            }
        }

        // Display pressure data and status
        bool output0_status = gpio_get(DIGITAL_OUTPUT0_PIN);
        bool pump_status = gpio_get(L298_ENABLE_A_PIN);
        bool vent_valve_status = gpio_get(L298_ENABLE_B_PIN);
        
        // Get timestamp in milliseconds (since boot)
        uint64_t timestamp_ms = time_us_64() / 1000;
        
        // Output CSV line for data capture script (must start with "CSV,")
        printf("CSV,%llu,%.2f,%d,%d,%d,%d\n",
                timestamp_ms,
                pressure_pa,
                di0_state ? 1 : 0,
                di1_state ? 1 : 0,
                pump_status ? 1 : 0,
                vent_valve_status ? 1 : 0);
        
        // Also output human-readable status for debugging
        printf("Pressure: %.2f Pa | DI0: %s | DI1: %s | Pump: %s | Vent: %s | DO0: %s\n",
                pressure_pa,
                di0_state ? "HIGH" : "LOW",
                di1_state ? "HIGH" : "LOW",
                pump_status ? "ON" : "OFF",
                vent_valve_status ? "ON" : "OFF",
                output0_status ? "HIGH" : "LOW");
        sleep_ms(100);  // Read every 100ms for better responsiveness
    }
}

