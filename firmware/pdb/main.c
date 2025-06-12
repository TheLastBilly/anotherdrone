#include <avr/io.h>
// F_CPU frequency is defined in the makefile
#include <util/delay.h>

// LED is on pin 0 of PORTC
#define LED 0
#define DELAY_MS 500

int main()
{
    /* uint8_t high = 0; */
    /* uint16_t ms = 0; */

/*     // Use MCLKCTRLB to disable the clock divider to get a 20 MHz clock */
/*     // MCLKCTRLB is under CCP (configuration change protection), so we */
/*     // have to write 0xD8 to the CCP register before we change it. */
    CCP = 0xD8;
    CLKCTRL_MCLKCTRLB &= ~(1);

/*     // setup LED pin for output in port C's direction register and set LED pin LOW */
    PORTA_DIR |= (1 << LED);
    PORTA_OUT &= ~(1 << LED);

    while (1)
    {
        // Toggle pin 0
        PORTA_OUT ^= PIN0_bm;

        _delay_ms(500);
    }

    return 0;
}

