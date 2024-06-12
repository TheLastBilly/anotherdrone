#include "Arduino.h"

#include "PID_v1.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define MPU6050_ADDRESS     0x69
#define SERIAL_RX_PIN       PB11
#define SERIAL_TX_PIN       PB10
#define COMP_FILTER_BIAS    0.08
#define SERIAL_BAUDRATE     115200
#define SERIAL              thisSerial
#define print               SERIAL.printf
#define CALIBRATION_LOOPS   5

typedef struct
{
    float roll, pitch, yaw;
} pitchRollYaw_t;

typedef struct
{
    pitchRollYaw_t gyro;
    pitchRollYaw_t accel;
    pitchRollYaw_t final;
    float lastUpdate;
} orientation_t;

static MPU6050 mpu(MPU6050_ADDRESS);
static orientation_t orientation;
static HardwareSerial SERIAL(SERIAL_RX_PIN, SERIAL_TX_PIN);

static inline float
fmillis( void )
{
    return ((float)millis())/1000.0;
}

static void 
updateOrientation( void )
{
    volatile float dt = 0;
    int16_t ax = 0, ay = 0, az = 0, 
            gx = 0, gy = 0, gz = 0;
    
    dt = fmillis() - orientation.lastUpdate;
    orientation.lastUpdate = fmillis();

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    orientation.gyro.roll += gx * dt;
    orientation.gyro.pitch += gy * dt;
    orientation.gyro.yaw += gz * dt;

    orientation.accel.roll = (atan2(-ay, az)*180.0)/M_PI;
    orientation.accel.pitch = (atan2(ax, sqrt(ay*ay + az*az))*180.0)/M_PI;

    orientation.final.roll = orientation.gyro.roll * COMP_FILTER_BIAS + orientation.accel.roll*(1-COMP_FILTER_BIAS);
    orientation.final.pitch = orientation.gyro.pitch * COMP_FILTER_BIAS + orientation.accel.pitch*(1-COMP_FILTER_BIAS);
    orientation.final.yaw = orientation.gyro.yaw;
}

void
setup( void )
{
    SERIAL.begin(SERIAL_BAUDRATE);
    Wire.begin();
    mpu.initialize();

    do
    {
        if(mpu.dmpInitialize() != 0)
        {
            print("Failed to initialize DMP\n\r");
            delay(500);
            continue;
        }
        mpu.CalibrateAccel(CALIBRATION_LOOPS);
        mpu.CalibrateGyro(CALIBRATION_LOOPS);
        print("MPU calibrated with %d loops\n\r", CALIBRATION_LOOPS);
        break;
    } while(true);
}

void
loop( void )
{
    updateOrientation();
    print("roll: %d, pitch: %d, yaw: %d\n\r", (int)orientation.final.roll, (int)orientation.final.pitch, (int)orientation.final.yaw);
}