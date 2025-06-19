#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// LED is on pin 0 of PORTC
#define LED 0
#define DELAY_MS 500

#define ioreg()                 CCP = CCP_IOREG_gc
#define XPIN_CONTROL(_p, _n)    PORT ## _p ## _PIN ## _n ## CTRL
#define PIN_CONTROL(_p, _n)     XPIN_CONTROL(_p, _n)
#define XPIN_DIRSET(_p)         PORT ## _p ## _DIRSET
#define PIN_DIRSET(_p)          XPIN_DIRSET(_p)
#define XPIN_DIRCLR(_p)         PORT ## _p ## _DIRCLR
#define PIN_DIRCLR(_p)          XPIN_DIRCLR(_p)
#define XPIN_OUTSET(_p)         PORT ## _p ## _OUTSET
#define PIN_OUTSET(_p)          XPIN_OUTSET(_p)
#define XPIN_OUTCLR(_p)         PORT ## _p ## _OUTCLR
#define PIN_OUTCLR(_p)          XPIN_OUTCLR(_p)
#define SET_PIN_INPUT(_p, _n)   PIN_DIRSET(_p) = _BV(_n); 
#define SET_PIN_OUTPUT(_p, _n)  PIN_DIRCLR(_p) = _BV(_n); 
#define XPORT_IN(_p)            PORT ## _p ## _IN 
#define PORT_IN(_p)             XPORT_IN(_p)
#define PIN_VALUE(_p, _n)       ((PORT_IN(_p) & _BV(_n)) > 0)
#define XPIN_INTFLAGS(_p)       PORT ## _p ## _INTFLAGS
#define PIN_INTFLAGS(_p)        XPIN_INTFLAGS(_p)
#define PIN_INT_SET(_p, _n)     ((PIN_INTFLAGS(_p) & _BV(_n)) > 0)

#define status_led_on()         PIN_OUTSET(STATUS_LED_PORT) = _BV(STATUS_LED_PIN) 
#define status_led_off()        PIN_OUTCLR(STATUS_LED_PORT) = _BV(STATUS_LED_PIN) 
#define control_int_set()       PIN_INT_SET(CONTROL_BTN_PORT, CONTROL_BTN_PIN)

// ESC GPIO Configuration
#define ESCA_PORT               A
#define ESCA_PIN                1
#define ESCB_PORT               A
#define ESCB_PIN                2
#define ESCC_PORT               A
#define ESCC_PIN                3
#define ESCD_PORT               A
#define ESCD_PIN                4

#define STATUS_LED_PORT         A
#define STATUS_LED_PIN          5

#define CONTROL_BTN_PORT        A
#define CONTROL_BTN_PIN         7

static volatile uint8_t control_btn = 0;

static void handle_control_btn();

ISR(PORTA_PORT_vect) 
{
    if (control_int_set()) {
        handle_control_btn();
    }    
}

static void
handle_control_btn()
{
    control_btn = ~control_btn;
    if (control_btn > 0) {
        status_led_on();
    } else {
        status_led_off();
    }
}

static void 
setup_clock()
{
    // Disable clock out and use internal oscillator
    ioreg();
    CLKCTRL_MCLKCTRLA = 0x00;

    // Disable prescaler division
    ioreg();
    CLKCTRL_MCLKCTRLB &= ~(CLKCTRL_PEN_bm);
}

static void
setup_gpio()
{
    // Configure port A (which has ESCs, Status LED, Reset button,
    // control button and battery status input)
    
    // ESC pins
    SET_PIN_OUTPUT(ESCA_PORT, ESCA_PIN);
    PIN_CONTROL(ESCA_PORT, ESCA_PIN) = PORT_ISC_INTDISABLE_gc;

    SET_PIN_OUTPUT(ESCB_PORT, ESCB_PIN);
    PIN_CONTROL(ESCB_PORT, ESCB_PIN) = PORT_ISC_INTDISABLE_gc;

    SET_PIN_OUTPUT(ESCC_PORT, ESCC_PIN);
    PIN_CONTROL(ESCC_PORT, ESCC_PIN) = PORT_ISC_INTDISABLE_gc;

    SET_PIN_OUTPUT(ESCD_PORT, ESCD_PIN);
    PIN_CONTROL(ESCD_PORT, ESCD_PIN) = PORT_ISC_INTDISABLE_gc;

    // Buttons
    //  Control Button
    SET_PIN_INPUT(ESCD_PORT, ESCD_PIN);
    PIN_CONTROL(ESCD_PORT, ESCD_PIN) = PORT_ISC_FALLING_gc;

    // LEDs
    //  Status LED
    SET_PIN_OUTPUT(STATUS_LED_PORT, STATUS_LED_PIN);
    PIN_CONTROL(STATUS_LED_PORT, STATUS_LED_PIN) = PORT_ISC_INTDISABLE_gc;
}

int main()
{
    setup_clock();
    sei();
    setup_gpio();

    TCA0.SINGLE.PER = 0xffff;
    /* TCA0.SINGLE.INTCTRL |= */ 
    TCA0.SINGLE.CTRLA = 0x07;

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

