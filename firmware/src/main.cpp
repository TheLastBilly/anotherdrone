#include "Arduino.h"

#include "PID_v1.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define MPU6050_ADDRESS     0x68
#define SERIAL_RX_PIN       PB11
#define SERIAL_TX_PIN       PB10
#define COMP_FILTER_BIAS    0.08
#define SERIAL_BAUDRATE     115200
#define SERIAL              thisSerial
#define print               SERIAL.printf
#define CALIBRATION_LOOPS   5
#define DEFAULT_KP          0.5
#define DEFAULT_KD          0
#define DEFAULT_KI          0
#define PASSES_PER_READING  10
#define YAW_DRIFT           1

static inline double fmillis();

typedef struct
{
    float roll, pitch, yaw;
} rollPitchYaw_t;

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

static MPU6050 mpu(MPU6050_ADDRESS);
static orientation_t orientation;
static HardwareSerial SERIAL(SERIAL_RX_PIN, SERIAL_TX_PIN);
static control_t control;
static rollPitchYaw_t target = {0};
static rollPitchYaw_t motorDelta = {0};

static inline double
fmillis( void )
{
    return ((double)millis())/1000.0;
}

void
setup( void )
{
    SERIAL.begin(SERIAL_BAUDRATE);
    // Wire.setSDA(PB7);
    // Wire.setSCL(PB6);
    // Wire.begin();
    // mpu.initialize();

    // do
    // {
    //     if(mpu.dmpInitialize() != 0)
    //     {
    //         print("Failed to initialize DMP\n\r");
    //         delay(1000);
    //         HAL_NVIC_SystemReset();
    //     }
    //     mpu.CalibrateAccel(CALIBRATION_LOOPS);
    //     mpu.CalibrateGyro(CALIBRATION_LOOPS);
    //     print("MPU calibrated with %d loops\n\r", CALIBRATION_LOOPS);
    //     break;
    // } while(true);

    // control.init();
}

// typedef struct
// {
//     int32_t x, y, z;
//     bool abort;
// } __attribute__((packed)) controllerData_t;

// static controllerData_t c = {0};

int16_t inputValues[4] = {0};
char buffer[100] = {0};
uint bufferLen = 0;

void
loop( void )
{
    // orientation.update(&mpu);
    // control.update(&orientation, &target, &motorDelta);
    // print("current:         roll: %d, pitch: %d, yaw: %d\n\r", (int)orientation.final.roll, (int)orientation.final.pitch, (int)orientation.final.yaw);
    // print("target:          roll: %d, pitch: %d, yaw: %d\n\r", (int)target.roll, (int)target.pitch, (int)target.yaw);
    // print("control bias:    roll: %d, pitch: %d, yaw: %d\n\r", (int)motorDelta.roll, (int)motorDelta.pitch, (int)motorDelta.yaw);

    while(SERIAL.available() > 0)
    {
        char c = 0;
        c = SERIAL.read();

        if(bufferLen >= sizeof(buffer))
        {
            memset(buffer, 0, sizeof(buffer));
            bufferLen = 0;
        }
        if(bufferLen == 0)
        {
            if(c == 'p')
            {
                buffer[bufferLen++] = c;
            }
        }
        else
        {
            buffer[bufferLen++] = c;
        }
    }


    if(buffer[0] == 'p' && buffer[1] == 't')
    {
        String str = String(buffer);
        if(str[0] == 'p' && str[1] == 't')
        {
            for(int i = 0; i < str.length(); i++)
                buffer[i] = str[i];

            int last = 0, count = 0;
            for(int i = 0; i < str.length(); i++)
            {
                if(str[i] != ',')
                    continue;

                inputValues[count++] = int((str.substring(last, i-1).toFloat() * 100));
                last = i + 1;
            }

            SERIAL.flush();
            print("%s\r\n", buffer);
        }
    }

    delay(1);
    // print("x: %d, y: %d, z: %d, abort: %d\r\n", inputValues[0], inputValues[1], inputValues[2], inputValues[3]);
}