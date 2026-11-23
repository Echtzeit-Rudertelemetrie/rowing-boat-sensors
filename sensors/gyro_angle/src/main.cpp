
#ifdef ARDUINO_NodeMCU_32S
// node-spezifisch
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

// later not yet relevant
#include <MadgwickAHRS.h>
#include <Kalman.h>
#elif defined(ARDUINO_ESP32_DEV)
#include <Arduino.h>
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Rowing stroke geometry
// Servo 0°  = FINISH — oar pointing forward/upward, blade just exited water
// Servo 90° = CATCH  — oar angled forward, blade entering water
// ─────────────────────────────────────────────────────────────────────────────
const float ANGLE_FINISH_DEG = 0.0f;
const float ANGLE_CATCH_DEG = 90.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Stroke timing (~20 spm training pace)
// Recovery: blade above water, linear sweep
// Drive:    blade in water, sine ease-in-out
// ─────────────────────────────────────────────────────────────────────────────
const unsigned long RECOVERY_DURATION_MS = 2000;
const unsigned long DRIVE_DURATION_MS = 1000;

// ─────────────────────────────────────────────────────────────────────────────
// Sensor sampling
// 100 Hz polling — MPU6050 DLPF limits meaningful bandwidth to 21 Hz
// ─────────────────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_INTERVAL_MS = 10;
const int SERVO_PIN = 13;

Adafruit_MPU6050 mpu;
Servo rowingServo;
float currentAngleDeg = ANGLE_FINISH_DEG;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float easeInOut(float t)
{
    return (1.0f - cosf(t * PI)) * 0.5f;
}

void calibrateGyro(int samples = 2000)
{
    sensors_event_t a, g, temp;
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; i++)
    {
        mpu.getEvent(&a, &g, &temp); // neue Messung pro Iteration
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(1);
    }
    gyroOffsetX = sumX / samples;
    gyroOffsetY = sumY / samples;
    gyroOffsetZ = sumZ / samples;
}

void sampleAndPlot()
{
    static unsigned long lastSample = 0;
    static unsigned long lastGyroUpdate = 0;
    static float winkel = 0.0f;
    static bool firstRun = true;

    unsigned long now = micros();

    // Prevent massive dt spike on the very first run after setup()
    if (firstRun)
    {
        lastGyroUpdate = now;
        lastSample = millis();
        firstRun = false;
    }

    // 1. Calculate dt
    float dt = (now - lastGyroUpdate) / 1000000.0f; // µs → s
    lastGyroUpdate = now;

    // 2. Read sensor and integrate EXACTLY ONCE per call
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    g.gyro.x -= gyroOffsetX;
    winkel += g.gyro.x * dt;

    // 3. Print data only every SAMPLE_INTERVAL_MS (100 Hz)
    unsigned long nowMs = millis();
    if (nowMs - lastSample >= SAMPLE_INTERVAL_MS)
    {
        lastSample = nowMs;
        Serial.print(">");
        Serial.print("Gyro_x: ");
        Serial.print(g.gyro.x, 4);
        Serial.print(",");
        Serial.print("Winkel_x: ");
        Serial.print(winkel * (180.0f / PI), 4);
        Serial.print(",");

        /*  Serial.print("Gyro_y: ");
         Serial.print(g.gyro.y, 4);
         Serial.print(",");
         Serial.print("Gyro_z: ");
         Serial.print(g.gyro.z, 4);
         Serial.print(",");
         Serial.print("Acce_x: ");
         */
        
        Serial.print(a.acceleration.x, 4);
        Serial.print(",");
        Serial.print("Acce_y: ");
        Serial.print(a.acceleration.y, 4);
        Serial.print(",");
        Serial.print("Acce_z: ");
        Serial.print(a.acceleration.z, 4);
        Serial.print(",");
        Serial.print("Servo_deg:");
        Serial.print(currentAngleDeg, 1);
        Serial.println();
    }
}

void recoveryPhase()
{
    unsigned long start = millis();
    while (millis() - start < RECOVERY_DURATION_MS)
    {
        float t = (float)(millis() - start) / (float)RECOVERY_DURATION_MS;
        currentAngleDeg = ANGLE_FINISH_DEG + t * (ANGLE_CATCH_DEG - ANGLE_FINISH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot();
    }
}

void drivePhase()
{
    unsigned long start = millis();
    while (millis() - start < DRIVE_DURATION_MS)
    {
        float t = (float)(millis() - start) / (float)DRIVE_DURATION_MS;
        float progress = easeInOut(t);
        currentAngleDeg = ANGLE_CATCH_DEG + progress * (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot();
    }
}

void setup()
{
    Serial.begin(115200);

    rowingServo.attach(SERVO_PIN);
    rowingServo.write((int)ANGLE_FINISH_DEG);
    delay(500);

    if (!mpu.begin())
    {
        while (1)
            delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);

    calibrateGyro();
    delay(100);
}

void loop()
{
    recoveryPhase();
    drivePhase();
}