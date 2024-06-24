#include "Arduino.h"

#include "PID_v1.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "SerialTransfer.h"
#include "Servo.h"
#include "ESC.h"

#define MPU6050_ADDRESS     0x68
#define SERIAL_RX_PIN       PB11
#define SERIAL_TX_PIN       PB10
#define COMP_FILTER_BIAS    0.08
#define SERIAL_BAUDRATE     115200
#define HOST_SERIAL         thisSerial
#define print               HOST_SERIAL.printf
#define CALIBRATION_LOOPS   5
#define DEFAULT_KP          0.5
#define DEFAULT_KD          0
#define DEFAULT_KI          0
#define PASSES_PER_READING  10
#define YAW_DRIFT           1
#define ESCA_PIN            PB4
#define ESCB_PIN            PB5
#define ESCC_PIN            PB3
#define ESCD_PIN            PA15

#define MANUAL_CONTROL

static inline double fmillis();

typedef struct
{
    float roll, pitch, yaw;
} rollPitchYaw_t;

#ifndef MANUAL_CONTROL
typedef struct
{
public:
    rollPitchYaw_t gyro;
    rollPitchYaw_t accel;
    rollPitchYaw_t final;
    float lastUpdate;

public:
    void update( MPU6050 * mpu )
    {
        int i = 0;
        volatile double dt = 0;
        int16_t ax = 0, ay = 0, az = 0, 
                gx = 0, gy = 0, gz = 0;
        int16_t dax = 0, day = 0, daz = 0, 
                dgx = 0, dgy = 0, dgz = 0;
        
        dt = fmillis() - this->lastUpdate;
        this->lastUpdate = fmillis();

        for(i = 0; i < PASSES_PER_READING; i++)
        {
            mpu->getMotion6(&dax, &day, &daz, &dgx, &dgy, &dgz);
            ax += dax;
            ay += day;
            az += daz;
            gx += dgx;
            gy += dgy;
            gz += dgz;
        }
        ax = dax/PASSES_PER_READING;
        ay = day/PASSES_PER_READING;
        az = daz/PASSES_PER_READING;
        gx = dgx/PASSES_PER_READING;
        gy = dgy/PASSES_PER_READING;
        gz = dgz/PASSES_PER_READING;

        this->gyro.roll += gx * dt;
        this->gyro.pitch += gy * dt;
        this->gyro.yaw += gz * dt;

        this->accel.roll = (atan2(-ay, az)*180.0)/M_PI;
        this->accel.pitch = (atan2(ax, sqrt(ay*ay + az*az))*180.0)/M_PI;

        this->final.roll = this->gyro.roll * COMP_FILTER_BIAS + this->accel.roll*(1-COMP_FILTER_BIAS);
        this->final.pitch = this->gyro.pitch * COMP_FILTER_BIAS + this->accel.pitch*(1-COMP_FILTER_BIAS);
        this->final.yaw = this->gyro.yaw - YAW_DRIFT*dt;
    }
} orientation_t;

typedef struct
{
public:
    double set, input, output;
    PID *pid;

public:
    void init( double kp, double ki, double kd )
    {
        if(this->pid != nullptr)
            delete this->pid;
        this->pid = new PID( &this->input, &this->output, &this->set, kp, ki, kd, DIRECT );
        this->pid->SetMode(AUTOMATIC);
    }

    double compute( double input, double set )
    {
        if(this->pid == nullptr)
            return this->output;
        
        this->input = input;
        this->set = set;
        this->pid->Compute();
        
        return this->output;
    }
} axisController_t;

typedef struct
{
public:
    axisController_t roll;
    axisController_t pitch;
    axisController_t yaw;

public:
    void init(double kp = DEFAULT_KP, double ki = DEFAULT_KI, double kd = DEFAULT_KD)
    {
        this->roll.init(kp, ki, kd);
        this->pitch.init(kp, ki, kd);
        this->yaw.init(kp, ki, kd);
    }

    void update( orientation_t * orientation, rollPitchYaw_t * target, rollPitchYaw_t * motor )
    {
        if(orientation == nullptr || target == nullptr || motor == nullptr)
            return;
        
        motor->roll = this->roll.compute(orientation->final.roll, target->roll);
        motor->pitch = this->pitch.compute(orientation->final.pitch, target->pitch);
        motor->yaw = this->yaw.compute(orientation->final.yaw, target->yaw);
    }

} control_t;
#endif

typedef struct
{
// A B
// C D
public:
    int pinA, pinB, pinC, pinD;
    ESC escA, escB, escC, escD;
    float speedA, speedB, speedC, speedD;

public:
    void init(int pinA, int pinB, int pinC, int pinD)
    {
        this->pinA = pinA;
        this->pinB = pinB;
        this->pinC = pinC;
        this->pinD = pinD;

        escA.attach(this->pinA);
        escB.attach(this->pinB);
        escC.attach(this->pinC);
        escD.attach(this->pinD);
    }

    void update(float throttle, float pitch, float roll, float yaw)
    {
        speedA = speedB = speedC = speedD = 0.f;
        throttle = ((throttle + 1.0)/2)/3.f;

        // Pitch
        pitch = (pitch + 1.f)/2.f;
        float pitchSpeed = throttle * pitch;
        speedC += pitchSpeed;
        speedD += pitchSpeed;
        float pitchSpeedInv = throttle - pitchSpeed;
        speedA += pitchSpeedInv;
        speedB += pitchSpeedInv;

        // Roll
        roll = (roll + 1.f)/2.f;
        float rollSpeed = throttle * roll;
        speedB += rollSpeed;
        speedD += rollSpeed;
        float rollSpeedInv = throttle - rollSpeed;
        speedA += rollSpeedInv;
        speedC += rollSpeedInv;

        // Yaw
        yaw = (yaw + 1.f)/2.f;
        float yawSpeed = throttle * yaw;
        speedB += yawSpeed;
        speedC += yawSpeed;
        float yawSpeedInv = throttle - yawSpeed;
        speedA += yawSpeedInv;
        speedD += yawSpeedInv;

        // escA.setSpeed(int(speedA*100));
        // escB.setSpeed(int(speedB*100));
        // escC.setSpeed(int(speedC*100));
        // escD.setSpeed(int(speedD*100));
    }

} motorController_t;

#ifndef MANUAL_CONTROL
static MPU6050 mpu(MPU6050_ADDRESS);
static orientation_t orientation;
static control_t control;
static rollPitchYaw_t target = {0};
static rollPitchYaw_t motorDelta = {0};
#else
static SerialTransfer manualTransfer;
#endif
static motorController_t motorController;
static HardwareSerial HOST_SERIAL(SERIAL_RX_PIN, SERIAL_TX_PIN);

static inline double
fmillis( void )
{
    return ((double)millis())/1000.0;
}

void
setup( void )
{
    pinMode(LED_BUILTIN, OUTPUT);
    HOST_SERIAL.begin(SERIAL_BAUDRATE);
    motorController.init(ESCA_PIN, ESCB_PIN, ESCC_PIN, ESCD_PIN);

#ifndef MANUAL_CONTROL
    Wire.begin();
    mpu.initialize();

    do
    {
        if(mpu.dmpInitialize() != 0)
        {
            print("Failed to initialize DMP\n\r");
            delay(1000);
            HAL_NVIC_SystemReset();
        }
        mpu.CalibrateAccel(CALIBRATION_LOOPS);
        mpu.CalibrateGyro(CALIBRATION_LOOPS);
        print("MPU calibrated with %d loops\n\r", CALIBRATION_LOOPS);
        break;
    } while(true);

    control.init();
#else
    manualTransfer.begin(HOST_SERIAL);
#endif
}

#ifdef MANUAL_CONTROL
typedef struct
{
    int32_t abort;
    float pitch, roll, yaw, throttle;
} __attribute__((packed)) manualData_t;
volatile static manualData_t manualData = {0};
#endif

void
loop( void )
{
#ifndef MANUAL_CONTROL
    orientation.update(&mpu);
    control.update(&orientation, &target, &motorDelta);
    print("current:         roll: %d, pitch: %d, yaw: %d\n\r", (int)orientation.final.roll, (int)orientation.final.pitch, (int)orientation.final.yaw);
    print("target:          roll: %d, pitch: %d, yaw: %d\n\r", (int)target.roll, (int)target.pitch, (int)target.yaw);
    print("control bias:    roll: %d, pitch: %d, yaw: %d\n\r", (int)motorDelta.roll, (int)motorDelta.pitch, (int)motorDelta.yaw);
#else
    if(manualTransfer.available() >= sizeof(manualData))
    {
        static bool ledPinOn = false;
        static volatile float inputTest = 0;
        uint16_t readSize = 0;
        readSize = manualTransfer.rxObj(manualData, readSize);
        motorController.update(manualData.throttle, manualData.pitch, manualData.roll, manualData.yaw);
        manualTransfer.reset();
        digitalWrite(LED_BUILTIN, ledPinOn);
        ledPinOn = !ledPinOn;

        static float speeds[4] = {0};
        speeds[0] = motorController.speedA;
        speeds[1] = motorController.speedB;
        speeds[2] = motorController.speedC;
        speeds[3] = motorController.speedD;
        uint16_t txSize = manualTransfer.txObj(speeds);
        manualTransfer.sendData(txSize);
    }

#endif
}