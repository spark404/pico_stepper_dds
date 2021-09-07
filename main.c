#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

#include "timer.pio.h"
#include "shift_register.pio.h"

// BOARD
#define SR1_SER 17
#define SR1_SCK 18
#define SR1_RCK 19

#define SR2_SER 14
#define SR2_SCK 15
#define SR2_RCK 16

#define I2C_SDC 21
#define I2C_SDA 20

#define SIGNAL_RESET 0
#define SIGNAL_SAFETY_DOOR 1
#define SIGNAL_CYCLE_START 2
#define SIGNAL_FEED_HOLD 3

#define LIMIT_X 4
#define LIMIT_Y 5
#define LIMIT_Z 6
#define LIMIT_A 7

#define CONTROL_SPINDLE_PWM 26  //A0

typedef union {
    struct {
        uint8_t x_step: 1;
        uint8_t y_step: 1;
        uint8_t z_step: 1;
        uint8_t a_step: 1;
        uint8_t x_dir: 1;
        uint8_t y_dir: 1;
        uint8_t z_dir: 1;
        uint8_t a_dir: 1;
    };
    uint8_t raw;
} stepper_state_t;

static PIO timer_pio = pio0;
static PIO shift_pio = pio1;
static int led_state = 1;

volatile uint sr_sm;

volatile int32_t phase = 0;
volatile int32_t frequency = 0;
int32_t stepper_speed = 0;
int32_t target_speed = 1000;
stepper_state_t stepper_state;

void step(int direction) {
    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);

    stepper_state.x_step =! stepper_state.x_step;
    stepper_state.x_dir = direction > 0 ? 1 : 0;
    shift_register_program_set(shift_pio, sr_sm, stepper_state.raw);

}

void pio_timer_isr() {
    timer_irq_clear(timer_pio);

    int32_t new_phase = phase + frequency;  // phase accumulate
    if ((phase ^ new_phase) < 0L)  // sign bit changed, time to step
    {
        if (frequency > 0L)
            step (1);
        else
            step (-1);
    }
    phase = new_phase;
}

/**
 * Set speed of the stepper motor to mm per minute
 * @param speed mm/min
 */
void set_stepper_speed_mmmin(int speed) {
#define STEPS_PER_ROTATION 200
#define PULLEY_TOOTH        25
#define MICROSTEPS           8
#define BELT_PITCH           2
    double mm_per_step = (double)PULLEY_TOOTH * MICROSTEPS * BELT_PITCH / STEPS_PER_ROTATION;
    double steps_per_mm = 1 / mm_per_step;
    double stepper_frequency = steps_per_mm * speed / 60;
    frequency = INT32_MAX / 100000 * stepper_frequency;
    printf("Stepper: speed %d, steps_per_mm %0.2f, stepper %0.2f hz, frequency %ld\n",
           speed, steps_per_mm, stepper_frequency, frequency);
}

int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "On-board LED"));

    stdio_init_all();

    // Setup the onboard led
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);

    // Setup i2c
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDC, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SDC);

    uint8_t rxdata;
    int ret = i2c_read_blocking(i2c_default, 0x50, &rxdata, 1, false);
    if (ret < 0) {
        printf("EEPROM not detected\n");
    } else {
        printf("EEPROM at 0x50\n");
    }


    uint offset = pio_add_program(timer_pio, &timer_program);
    uint sm = pio_claim_unused_sm(timer_pio, true);

    uint sr_offset = pio_add_program(shift_pio, &shift_register_program);
    sr_sm = pio_claim_unused_sm(shift_pio, true);

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_timer_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Pico clock runs at 125Mhz
    // This makes the PIO run at 10Mhz
    timer_program_init(timer_pio, sm, offset, 12.5f);

    shift_register_program_init(shift_pio, sm, sr_offset, SR1_SER, SR1_SCK);

    // Generate an IRQ signal every 1000 clocks
    // Effective rate 100Khz
    timer_set_period(timer_pio, sm, offset, 100);
    set_stepper_speed_mmmin(0);

    uint32_t leds = 1;
    uint8_t led_dir = 1;
    int integral = 0;
    int last_error = 0;
    for (;;) {
        leds = led_dir ? leds << 1 : leds >> 1;
        if (leds == 1 || leds == (1<<7)) {
            led_dir = !led_dir;
        }

        // PI(D) controller
        int error = target_speed - stepper_speed;
        integral += error;
        if (integral > target_speed) {
            integral = target_speed;
        } else if (integral < -target_speed) {
            integral = -target_speed;
        }
        int derivative = error - last_error;
        double kp = 0.1;
        double ki = 0.001;
        double kd = 0.1;

        stepper_speed += kp * error + (ki * integral) + (kd * derivative);
        if (stepper_speed == 1001) {
            target_speed = -1000;
        } else if (stepper_speed == -993) {
            target_speed = 1000;
        }
        set_stepper_speed_mmmin(stepper_speed);
        last_error = error;

        sleep_ms(250);
    }
}

