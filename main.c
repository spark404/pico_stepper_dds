#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"

#include "timer.pio.h"

static PIO pio = pio0;
static int led_state = 1;

void pio_timer_isr() {
    timer_irq_clear(pio);

    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
}

int main() {
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_LED_PIN, "On-board LED"));

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, led_state);

    uint offset = pio_add_program(pio, &timer_program);
    uint sm = pio_claim_unused_sm(pio, true);

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_timer_isr);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Pico clock runs at 125Mhz
    // This makes the PIO run at 10Mhz
    timer_program_init(pio, sm, offset, 12.5f);
    timer_set_period(pio, sm, offset, 2500000);

    for (;;) {
        sleep_ms(250);
    }
}

