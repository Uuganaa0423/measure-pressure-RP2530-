#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "XGZP6847D.h"

// XGZP6847D Pressure Sensor Configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define I2C_FREQ 100000  // 100 kHz
#define PRESSURE_SENSOR_ADDR 0x6D

// Sensor register map (from datasheet)
#define PRESSURE_MSB_REG 0x06  // Pressure Data out <23:16>
#define PRESSURE_CSB_REG 0x07  // Pressure Data out <15:8>
#define PRESSURE_LSB_REG 0x08  // Pressure Data out <7:0>
#define MEASUREMENT_COMMAND_REG 0x30

// Measurement-command bit-field helpers (see datasheet)
#define MEAS_SLEEP_TIME_SHIFT 4
#define MEAS_SLEEP_TIME_MASK  0xF0
#define MEAS_GAIN_SHIFT       3
#define MEAS_GAIN_MASK        0x38         // Gain_P<5:3>
#define MEAS_SCO_BIT          (1u << 3)    // Start of conversion flag
#define MEAS_CTRL_MASK        0x07         // Measurement_control<2:0>

// Sleep-time presets (active in sleep-mode conversion)
#define MEAS_SLEEP_0_MS       0x0
#define MEAS_SLEEP_62MS       0x1
#define MEAS_SLEEP_125MS      0x2
#define MEAS_SLEEP_250MS      0x3
#define MEAS_SLEEP_500MS      0x4
#define MEAS_SLEEP_1S         0xF

// Gain presets (table depends on datasheet, default to unity)
#define MEAS_GAIN_X1          0x0
#define MEAS_GAIN_X2          0x1
#define MEAS_GAIN_X4          0x2
#define MEAS_GAIN_X8          0x3

// Measurement-control field (per datasheet description)
#define MEAS_CTRL_PRESSURE_ONLY            0x001
#define MEAS_CTRL_COMBINED_SINGLE_SHOT     0x002  // Temperature + pressure, one shot
#define MEAS_CTRL_COMBINED_SLEEP_MODE      0x003  // Periodic combined conversion (sleep mode)
#define MEAS_CTRL_RESERVED                 0x007  // Example placeholder

// Helper macros to build command fields
#define MEAS_SLEEP_FIELD(code)   (((code) & 0x0F) << MEAS_SLEEP_TIME_SHIFT)
#define MEAS_GAIN_FIELD(code)    (((code) & 0x07) << MEAS_GAIN_SHIFT)
#define MEAS_CTRL_FIELD(code)    ((code) & MEAS_CTRL_MASK)

// Default command: immediate combined temperature + pressure conversion
#define DEFAULT_SLEEP_CODE      MEAS_SLEEP_0_MS          // Not used for single-shot
#define DEFAULT_GAIN_CODE       MEAS_GAIN_X1
#define DEFAULT_MEAS_CONTROL    MEAS_CTRL_COMBINED_SINGLE_SHOT
#define DEFAULT_MEAS_COMMAND    (MEAS_SLEEP_FIELD(DEFAULT_SLEEP_CODE) | \
                                 MEAS_GAIN_FIELD(DEFAULT_GAIN_CODE)   | \
                                 MEAS_CTRL_FIELD(DEFAULT_MEAS_CONTROL) | \
                                 MEAS_SCO_BIT)

// L298N Motor Driver Configuration for Vacuum Pump
#define DIGITAL_INPUT0_PIN 2      // GPIO2: Control signal (LOW -> HIGH starts pump)
#define DIGITAL_OUTPUT0_PIN 5     // GPIO3: Output signal (HIGH when target pressure reached)
#define L298_IN1_PIN       28     // GPIO28: L298N IN1 pin
#define L298_IN2_PIN       29     // GPIO29: L298N IN2 pin
#define L298_ENABLE_A_PIN  15     // GPIO15: L298N Enable_A pin (ENA)

// L298N Motor Driver Configuration for Vent Valve
#define L298_IN3_PIN       11     // GPIO11: L298N IN3 pin (vent valve)
#define L298_IN4_PIN       12     // GPIO12: L298N IN4 pin (vent valve)

// Target pressure level: -60kPa = -57764 Pa /according to measurements/
#define TARGET_PRESSURE_PA -57764.0f

/**
 * Initialize I2C interface for pressure sensor
 */
void init_i2c_pressure_sensor(void)
{
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

/**
 * Write raw command byte to measurement-command register (0x30)
 */
bool write_measurement_command(uint8_t command)
{
    uint8_t buffer[2] = {MEASUREMENT_COMMAND_REG, command};
    int written = i2c_write_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, buffer, sizeof(buffer), false);
    return written == sizeof(buffer);
}

/**
 * Read a single sensor register
 */
bool read_sensor_register(uint8_t reg_addr, uint8_t *value)
{
    if (!value) {
        return false;
    }

    int ret = i2c_write_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, &reg_addr, 1, true);
    if (ret != 1) {
        return false;
    }

    ret = i2c_read_blocking(I2C_PORT, PRESSURE_SENSOR_ADDR, value, 1, false);
    return ret == 1;
}

/**
 * Start a pressure/temperature conversion and wait for SCO to clear
 */
bool trigger_measurement(uint32_t timeout_ms)
{
    if (!write_measurement_command(DEFAULT_MEAS_COMMAND)) {
        printf("Failed to write measurement command (0x%02X)\n", DEFAULT_MEAS_COMMAND);
        return false;
    }

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    uint8_t status = 0;

    do {
        if (!read_sensor_register(MEASUREMENT_COMMAND_REG, &status)) {
            printf("Failed to read measurement command register\n");
            return false;
        }

        if ((status & MEAS_SCO_BIT) == 0) {
            // Conversion finished
            return true;
        }

        sleep_ms(1);
    } while (!time_reached(deadline));

    printf("Measurement timeout: SCO bit stayed high. Last status=0x%02X\n", status);
    return false;
}

/**
 * Read pressure from XGZP6847D sensor
 * @return 24-bit pressure value (raw sensor reading)
 */
int32_t read_pressure_raw(void)
{
    uint8_t msb = 0, csb = 0, lsb = 0;

    if (!read_sensor_register(PRESSURE_MSB_REG, &msb) ||
        !read_sensor_register(PRESSURE_CSB_REG, &csb) ||
        !read_sensor_register(PRESSURE_LSB_REG, &lsb)) {
        printf("Failed to read pressure registers\n");
        return 0;
    }

    int32_t pressure = ((int32_t)msb << 16) | ((int32_t)csb << 8) | (int32_t)lsb;

    // Handle sign extension for 24-bit signed value
    if (pressure & 0x00800000) {
        pressure |= 0xFF000000;
    }

    return pressure;
}

/**
 * Dump pressure/temperature registers for debugging
 */
void dump_sensor_data_registers(void)
{
    uint8_t value = 0;

    printf("--- Sensor Register Snapshot ---\n");
    for (uint8_t addr = PRESSURE_MSB_REG; addr <= PRESSURE_LSB_REG; addr++) {
        if (read_sensor_register(addr, &value)) {
            printf("Reg 0x%02X = 0x%02X\n", addr, value);
        } else {
            printf("Reg 0x%02X read failed\n", addr);
        }
    }

    if (read_sensor_register(MEASUREMENT_COMMAND_REG, &value)) {
        printf("Reg 0x%02X (CMD) = 0x%02X\n", MEASUREMENT_COMMAND_REG, value);
    } else {
        printf("Reg 0x%02X (CMD) read failed\n", MEASUREMENT_COMMAND_REG);
    }
    printf("--------------------------------\n");
}

/**
 * Convert raw pressure reading to Pascal (Pa)
 * Using k-factor = 16 for conversion
 * @param raw_value Raw 24-bit pressure value from sensor
 * @return Pressure in Pascal
 */
float convert_to_pascal(int32_t raw_value)
{
    // XGZP6847D conversion: Pressure (Pa) = raw_value / k
    // k-factor = 16
    const float k_factor = 16.0f;
    return (float)raw_value / k_factor;
}


/**
 * Monitor pressure readings via USB UART
 */
void monitor_pressure(void)
{
    if (!trigger_measurement(50)) {
        printf("Measurement trigger failed\n");
        return;
    }

    dump_sensor_data_registers();

    int32_t raw_pressure = read_pressure_raw();
    float pressure_pa = convert_to_pascal(raw_pressure);

    printf("Pressure - Raw: %d (0x%06X), Pa: %.2f\n",
            (int)raw_pressure, (unsigned int)(raw_pressure & 0xFFFFFF), pressure_pa);
}

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
    gpio_put(L298_IN3_PIN, 1);
    gpio_put(L298_IN4_PIN, 0);
    printf("Vent valve enabled\n");
}

/**
 * Disable vent valve by setting IN3=LOW, IN4=LOW
 */
static void disable_vent_valve(void)
{
    gpio_put(L298_IN3_PIN, 0);
    gpio_put(L298_IN4_PIN, 0);
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
        if (pressure_pa <= TARGET_PRESSURE_PA) {
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

