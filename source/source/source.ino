/*
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.
 */

// Stop button is attached to PIN 0 (IO0)
// #define BTN_STOP_ALARM 0
#include <PS4Controller.h>

#define TIMER_FREQUENCY (1000 * 1000 * 1)
#define ESC_FREQUENCY   50
#define PERIOD_TICKS    ((TIMER_FREQUENCY)/(ESC_FREQUENCY))
#define CUTOFF_TICKS    (PERIOD_TICKS/10)
#define PIN_INFO_COUNT  (sizeof(pin_infos)/sizeof(pin_infos[0]))
#define MAX_COUNTER     (~((uint64_t)0))
#define MIN_SPEED       50
#define MAX_SPEED       100
#define MAX_DELTA       50
#define TURN_DELTA      .5
#define SPEED_RANGE     (MAX_SPEED - MIN_SPEED)
#define UNSAFE_COLOR    255, 0, 0
#define SAFE_COLOR      0, 255, 0

static const float mixingMatrix[] =
{
//  Thrust  Pitch   Roll    Yaw
    1.0,    -1.0,   1.0,    1.0,
    1.0,    -1.0,   -1.0,   -1.0,
    1.0,    1.0,    1.0,    -1.0,
    1.0,    1.0,    -1.0,   1.0,
};


typedef enum TIMER_STATE_t {
    TIMER_STATE_RESET,
    TIMER_STATE_SET
} TIMER_STATE_t;

typedef struct pin_info_t {
    gpio_num_t pin;
    uint64_t at;
} pin_info_t;

pin_info_t pin_infos[] = {
    {GPIO_NUM_5, 0},
    {GPIO_NUM_2, 0},
    {GPIO_NUM_17, 0},
    {GPIO_NUM_27, 0}
};

uint64_t motor_delta[PIN_INFO_COUNT] = {
    0, 0, 0, 0
};

volatile TIMER_STATE_t timer_state = TIMER_STATE_RESET;
volatile uint current_pin = 0;
volatile uint current_time = 0;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t base_speed = 0;
volatile bool flight_ready = false;

void ARDUINO_ISR_ATTR onTimer();

static void set_all_pin_duty_cycle(uint64_t duty)
{
    uint i = 0, s = 0, curr = 0;
    pin_info_t infos[PIN_INFO_COUNT] = {};

    portENTER_CRITICAL_ISR(&timerMux);
    memcpy(infos, pin_infos, sizeof(infos));
    portEXIT_CRITICAL_ISR(&timerMux);

    for(i = 0; i < PIN_INFO_COUNT; i++) {
        infos[i].at = ((CUTOFF_TICKS) * duty)/100;
    }

    portENTER_CRITICAL_ISR(&timerMux);
    memcpy(pin_infos, infos, sizeof(pin_infos));
    portEXIT_CRITICAL_ISR(&timerMux);
}

static void set_pin_duty_cycle(gpio_num_t pin, uint64_t duty)
{
    uint i = 0, s = 0, curr = 0;
    pin_info_t infos[PIN_INFO_COUNT] = {};;

    if (duty > 100) {
        duty = 100;
    }

    portENTER_CRITICAL_ISR(&timerMux);
    memcpy(infos, pin_infos, sizeof(infos));
    portEXIT_CRITICAL_ISR(&timerMux);

    for(i = 0; i < PIN_INFO_COUNT; i++) {
        if (infos[i].pin == pin) {
            infos[i].at = ((CUTOFF_TICKS) * duty)/100;
            break;
        }
    }

    portENTER_CRITICAL_ISR(&timerMux);
    memcpy(pin_infos, infos, sizeof(pin_infos));
    portEXIT_CRITICAL_ISR(&timerMux);
}

static void run_timer_in(uint64_t ticks) {
    timerAlarm(timer, ticks, true, 0);
}

void ARDUINO_ISR_ATTR onTimer() {
    int i = 0, found = 0;
    uint64_t min = MAX_COUNTER;
    TIMER_STATE_t state, transition;
    pin_info_t infos[PIN_INFO_COUNT] = {};

    portENTER_CRITICAL_ISR(&timerMux);
    memcpy(infos, pin_infos, sizeof(infos));
    state = timer_state;
    portEXIT_CRITICAL_ISR(&timerMux);
    
    switch (state) {
    case TIMER_STATE_RESET:
        found = 0;
        for (i = 0; i < PIN_INFO_COUNT; i++)
        {
            if (infos[i].at > 0) {
                gpio_set_level(infos[i].pin, 1);
            } else
                gpio_set_level(infos[i].pin, 0);
        }
        transition = TIMER_STATE_SET;
        current_time = 0;
    case TIMER_STATE_SET:
        for (i = 0; i < PIN_INFO_COUNT; i++)
        {
            if (current_time == infos[i].at) {
                gpio_set_level(infos[i].pin, 0);
            } else if (current_time < infos[i].at && infos[i].at < min) {
                min = infos[i].at;
            }
        }

        if (min == MAX_COUNTER) {
            transition = TIMER_STATE_RESET;
            run_timer_in(PERIOD_TICKS - current_time);
        } else {
            transition = TIMER_STATE_SET;
            run_timer_in(min - current_time);
            current_time = min;
        }
        break;
    }

    portENTER_CRITICAL_ISR(&timerMux);
    if (timer_state == state) {
        timer_state = transition;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

static void set_speeds(float throttle, float pitch, float roll, float yaw)
{
    float speedA, speedB, speedC, speedD;
    speedA = throttle - throttle*pitch + throttle*roll + throttle*yaw;
    speedB = throttle +  throttle*pitch + throttle*roll - throttle*yaw;
    speedC = throttle - throttle*pitch - throttle*roll - throttle*yaw;
    speedD = throttle + throttle*pitch - throttle*roll + throttle*yaw;// + (1.f - pitch) + (1.f - roll) ;//+ mixingMatrix[15] * yaw);

    // speedA = (speedA + 2.0f) / 4.0f;
    if (speedA < 0.f) {
        speedA = 0;
    }
    // speedB = (speedB + 2.0f) / 4.0f;
    if (speedB < 0.f) {
        speedA = 0;
    }
    // speedC = (speedC + 2.0f) / 4.0f;
    if (speedC < 0.f) {
        speedA = 0;
    }
    // speedD = (speedD + 2.0f) / 4.0f;
    if (speedD < 0.f) {
        speedA = 0;
    }

    set_pin_duty_cycle(pin_infos[0].pin, MIN_SPEED + (uint64_t)(((float)SPEED_RANGE) * speedA));
    set_pin_duty_cycle(pin_infos[1].pin, MIN_SPEED + (uint64_t)(((float)SPEED_RANGE) * speedB));
    set_pin_duty_cycle(pin_infos[2].pin, MIN_SPEED + (uint64_t)(((float)SPEED_RANGE) * speedC));
    set_pin_duty_cycle(pin_infos[3].pin, MIN_SPEED + (uint64_t)(((float)SPEED_RANGE) * speedD));
}

void setup() {
    static volatile int i = 0;
    PS4.begin("2C:98:11:81:B8:0A");
    // PS4.begin();
    Serial.begin(115200);
//   return;

  // Set BTN_STOP_ALARM to input mode
//   pinMode(BTN_STOP_ALARM, INPUT_PULLUP);
    for (i = 0; i < PIN_INFO_COUNT; i++) {
        gpio_set_direction(pin_infos[i].pin, GPIO_MODE_OUTPUT);
        set_pin_duty_cycle(pin_infos[i].pin, 0);
    }
  // Set timer frequency to 1Mhz
  timer = timerBegin(TIMER_FREQUENCY);

  // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer);
    run_timer_in(1);
}

static void send_controller_status()
{
    PS4.setRumble(255,0);
    PS4.sendToController();
    delay(250);
    PS4.setRumble(0,0);
    PS4.sendToController();
    if (flight_ready) {
        PS4.setLed(UNSAFE_COLOR);
    } else {
        PS4.setLed(SAFE_COLOR);
    }
    PS4.sendToController();
}

void loop() {
    uint i = 0;
    uint64_t duty = 0;
    uint64_t raw = 0;
    static uint64_t past = 0;
    static uint64_t count = 0;

    if (PS4.isConnected()) {
        // duty = map(PS4.LStickY(), -128, 128, 50, 100);
        // if (duty != past) {
        //     set_pin_duty_cycle(GPIO_NUM_5, duty);
        //     set_pin_duty_cycle(GPIO_NUM_2, duty);
        //     past = duty;
        //     // Serial.print("Setting duty to ");
        //     delay(100);
        // }

        if (PS4.Cross()) {
            flight_ready = !flight_ready;
            if (flight_ready) {
                set_all_pin_duty_cycle(MIN_SPEED);
                send_controller_status();
                delay(2000);
                Serial.println("Flight ready");
            } else {
                set_all_pin_duty_cycle(MIN_SPEED);
                delay(2000);
                set_all_pin_duty_cycle(0);
                delay(2000);
                send_controller_status();
                Serial.println("Not flight ready");
            }
        }

        if (flight_ready) {
            set_speeds(
                ((float)map(PS4.LStickY(), -128, 128, 0, 100))/100.0f,
                ((float)map(PS4.RStickY(), -128, 128, -100, 100))/100.0f,
                ((float)map(PS4.RStickX(), -128, 128, -100, 100))/100.0f,
                (PS4.R1() ? TURN_DELTA : 0.0f) - (PS4.L1() ? TURN_DELTA : 0.0f)
            );
        }
        // count = 0;
        // if (past == duty) {
        //     past = duty;
        // }
    } else {
        set_all_pin_duty_cycle(MIN_SPEED);
    }
    delay((1000/60));
}
