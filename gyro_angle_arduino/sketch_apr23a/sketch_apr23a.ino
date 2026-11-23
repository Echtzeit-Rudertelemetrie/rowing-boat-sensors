#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ─────────────────────────────────────────────────────────────────────────────
// Rowing stroke geometry
// Reference: ~20 spm training pace; total oar arc ≈ 90°
//
// Servo 0°  = FINISH — oar pointing forward/upward, blade just exited water
// Servo 90° = CATCH  — oar angled forward, blade entering water
// ─────────────────────────────────────────────────────────────────────────────
const float         ANGLE_FINISH_DEG      = 0.0f;   // ° — oar at finish
const float         ANGLE_CATCH_DEG       = 90.0f;  // ° — oar at catch

// ─────────────────────────────────────────────────────────────────────────────
// Stroke timing
// Recovery: blade above water, linear angular velocity (relaxed, constant)
// Drive:    blade in water, non-linear (slow catch → fast mid → slight decel)
// ─────────────────────────────────────────────────────────────────────────────
const unsigned long RECOVERY_DURATION_MS  = 2000;   // ms
const unsigned long DRIVE_DURATION_MS     = 1000;   // ms

// ─────────────────────────────────────────────────────────────────────────────
// Sensor & plotter
// 115200 baud required — select in Serial Plotter dropdown
// 100 Hz sample rate: each line = one plotter data point
// MPU6050 DLPF at 21 Hz limits meaningful signal bandwidth to ~21 Hz,
// but 100 Hz polling avoids aliasing within that band
// ─────────────────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_INTERVAL_MS    = 10;     // ms → 100 Hz
const int           SERVO_PIN             = 13;

Adafruit_MPU6050 mpu;
Servo            rowingServo;
float            currentAngleDeg          = ANGLE_FINISH_DEG;

// ─────────────────────────────────────────────────────────────────────────────
// Sine ease-in-out: t ∈ [0.0, 1.0] → curved progress ∈ [0.0, 1.0]
// Applied to drive phase only.
// Approximates real drive: legs/back engage slowly, peak force mid-drive,
// arms finish with slight deceleration.
// ─────────────────────────────────────────────────────────────────────────────
float easeInOut(float t) {
    return (1.0f - cosf(t * PI)) * 0.5f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Sample MPU6050 and emit one plotter line, rate-limited to SAMPLE_INTERVAL_MS
// Format: tab-separated "Label:value" pairs — Arduino IDE 2 Serial Plotter
//         assigns one channel per unique label, keeps them separated
// ─────────────────────────────────────────────────────────────────────────────
void sampleAndPlot() {
    static unsigned long lastSample = 0;
    unsigned long now = millis();
    if (now - lastSample < SAMPLE_INTERVAL_MS) return;
    lastSample = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Gyroscope — angular velocity in rad/s around each IMU axis
    Serial.print("Gyro_X:");    Serial.print(g.gyro.x, 4);         Serial.print("\t");
    Serial.print("Gyro_Y:");    Serial.print(g.gyro.y, 4);         Serial.print("\t");
    Serial.print("Gyro_Z:");    Serial.print(g.gyro.z, 4);         Serial.print("\t");

    // Accelerometer — linear acceleration in m/s² (includes gravity)
    Serial.print("Accel_X:");   Serial.print(a.acceleration.x, 4); Serial.print("\t");
    Serial.print("Accel_Y:");   Serial.print(a.acceleration.y, 4); Serial.print("\t");
    Serial.print("Accel_Z:");   Serial.print(a.acceleration.z, 4); Serial.print("\t");

    // Current servo angle for correlation with sensor data
    Serial.print("Servo_deg:"); Serial.print(currentAngleDeg, 1);

    Serial.println(); // one \n = one plotter sample point
}

// ─────────────────────────────────────────────────────────────────────────────
// Recovery phase: FINISH → CATCH, linear sweep
// Blade is above water — relaxed, constant angular velocity
// ─────────────────────────────────────────────────────────────────────────────
void recoveryPhase() {
    unsigned long start = millis();
    while (millis() - start < RECOVERY_DURATION_MS) {
        float t = (float)(millis() - start) / (float)RECOVERY_DURATION_MS; // linear 0→1
        currentAngleDeg = ANGLE_FINISH_DEG + t * (ANGLE_CATCH_DEG - ANGLE_FINISH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Drive phase: CATCH → FINISH, sine ease-in-out sweep
// Blade is in water — slow at catch, accelerates through mid-drive
// ─────────────────────────────────────────────────────────────────────────────
void drivePhase() {
    unsigned long start = millis();
    while (millis() - start < DRIVE_DURATION_MS) {
        float t        = (float)(millis() - start) / (float)DRIVE_DURATION_MS;
        float progress = easeInOut(t);
        currentAngleDeg = ANGLE_CATCH_DEG + progress * (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot();
    }
}

void setup() {
    Serial.begin(115200); // must match Serial Plotter baud selection

    rowingServo.attach(SERVO_PIN);
    rowingServo.write((int)ANGLE_FINISH_DEG);
    delay(500);

    if (!mpu.begin()) {
        while (1) delay(10); // halt — sensor not detected
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
}

void loop() {
    recoveryPhase();
    drivePhase();
}