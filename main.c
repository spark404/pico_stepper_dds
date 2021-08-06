#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#include "timer.pio.h"
#include "shift_register.pio.h"

// Pinout of the 74HC595
#define SR_SERIN 15
#define SR_CLK   13

#define SIGNAL 12

static PIO timer_pio = pio0;
static PIO shift_pio = pio1;
static int led_state = 1;
static int signal_state = 1;

volatile int32_t phase = 0;
volatile int32_t frequency = 0;

void step(int direction) {
    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
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

int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "On-board LED"));

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);

    gpio_init(SIGNAL);
    gpio_set_dir(SIGNAL, GPIO_OUT);
    gpio_put(SIGNAL, signal_state);

    uint offset = pio_add_program(timer_pio, &timer_program);
    uint sm = pio_claim_unused_sm(timer_pio, true);

    uint sr_offset = pio_add_program(shift_pio, &shift_register_program);
    uint sr_sm = pio_claim_unused_sm(shift_pio, true);

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_timer_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Pico clock runs at 125Mhz
    // This makes the PIO run at 10Mhz
    timer_program_init(timer_pio, sm, offset, 12.5f);

    shift_register_program_init(shift_pio, sm, sr_offset, SR_SERIN, SR_CLK);

    // Generate an IRQ signal every 1000 clocks
    // Effective rate 10Khz
    timer_set_period(timer_pio, sm, offset, 1000);

    frequency = INT32_MAX / 10000;  // Effective 1hz

    uint32_t leds = 1;
    for (;;) {
        gpio_put(SIGNAL, signal_state);
        signal_state = !signal_state;

        shift_register_program_set(shift_pio, sr_sm, leds);
        uint32_t msb = (leds & (1 << 7));
        leds <<= 1;
        leds |= msb ? 0x1 : 0x0;
        sleep_ms(100);
    }
}

