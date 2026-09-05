#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// =====================================
// PINS Configurations
constexpr uint8_t SDA_PIN    = 19;
constexpr uint8_t SCL_PIN    = 18;

constexpr uint8_t SERVO_PIN  = 25;
constexpr uint8_t TRIG_PIN   = 32;
constexpr uint8_t ECHO_PIN   = 33;
constexpr uint8_t BUZZER_PIN = 13;
constexpr uint8_t LED_PIN    = 23;

// =====================================
// OLED Display Configurations
constexpr uint8_t SCREEN_WIDTH  = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_ADDRESS  = 0x3C;

// =====================================
// SERVO Configurations
constexpr int SERVO_MIN_ANGLE = 20;
constexpr int SERVO_MAX_ANGLE = 180;
constexpr int SERVO_STEP      = 2;

constexpr int SERVO_MIN_PULSE_US = 1000;
constexpr int SERVO_MAX_PULSE_US = 2000;

constexpr uint32_t SERVO_SETTLE_MS = 30;

// =====================================
// Ultrasonic Configurations
constexpr uint32_t ECHO_TIMEOUT_US = 30000;

// Approximate speed of sound in cm/us.
constexpr float SOUND_SPEED_CM_PER_US = 0.0343f;

// =====================================
// Alert Configurations
constexpr int BUZZER_MAX_DISTANCE_CM = 100;

constexpr int MIN_TRACKED_DISTANCE_CM = 5;

constexpr int MIN_BEEP_INTERVAL_MS = 100;
constexpr int MAX_BEEP_INTERVAL_MS = 1000;

constexpr int MIN_BUZZER_FREQUENCY_HZ = 700;
constexpr int MAX_BUZZER_FREQUENCY_HZ = 2500;

constexpr int BEEP_DURATION_MS = 25;

// =====================================
//LED
constexpr int LED_ALERT_DISTANCE_CM = 50;

// =====================================
// Global Objects

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo radarServo;

uint32_t lastBeepTimeMs = 0;

// =====================================
// Ultrasonic Function
int readDistanceCm() {
    // Sensor required a short low pulse.
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Trigger the burst.
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure time when echo pin goes HIGH.
    const uint32_t durationUs =
        pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

    // Timeout means no echo was received.
    if (durationUs == 0) {
        return -1;
    }

    // Sound goes from object and back, divide by 2.
    const float distanceCm =
        durationUs * SOUND_SPEED_CM_PER_US / 2.0f;

    return static_cast<int>(distanceCm);
}

// =====================================
// LED
void updateLed(const int distanceCm) {
    const bool objectIsClose =
        distanceCm > 0 &&
        distanceCm <= LED_ALERT_DISTANCE_CM;

    digitalWrite(
        LED_PIN,
        objectIsClose ? HIGH : LOW
    );
}

// =====================================
// Piezo Buzzer - Function to generate a chirp required due to tone() function causing issues with Servo library on ESP32.
void piezoChirp(const int frequencyHz, const int durationMs) {
    const int halfPeriodUs =
        1000000 / frequencyHz / 2;

    const int cycles =
        frequencyHz * durationMs / 1000;

    for (int i = 0; i < cycles; ++i) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(halfPeriodUs);

        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(halfPeriodUs);
    }

    digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer(const int distanceCm) {
    if (distanceCm <= 0 || distanceCm > BUZZER_MAX_DISTANCE_CM) { return; }

    const uint32_t nowMs = millis();

    // Constrain the distance to avoid extreme values affecting the beep interval and frequency.
    const int constrainedDistance = constrain(distanceCm, MIN_TRACKED_DISTANCE_CM, BUZZER_MAX_DISTANCE_CM);

    // Shoter beep intervals for closer objects.
    const int beepIntervalMs = map(
        constrainedDistance,
        MIN_TRACKED_DISTANCE_CM,
        BUZZER_MAX_DISTANCE_CM,
        MIN_BEEP_INTERVAL_MS,
        MAX_BEEP_INTERVAL_MS
    );

    if (nowMs - lastBeepTimeMs < beepIntervalMs) { return; }

    // Higher pitch for closer objects.
    const int frequencyHz = map(
        constrainedDistance,
        BUZZER_MAX_DISTANCE_CM,
        MIN_TRACKED_DISTANCE_CM,
        MIN_BUZZER_FREQUENCY_HZ,
        MAX_BUZZER_FREQUENCY_HZ
    );

    piezoChirp(frequencyHz, BEEP_DURATION_MS);

    lastBeepTimeMs = nowMs;
}

// =====================================
// OLED Display
void updateDisplay(const int angleDegrees, const int distanceCm) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("ULTRASONIC RADAR");

    display.setCursor(0, 20);
    display.print("Angle: ");
    display.print(angleDegrees);
    display.println(" deg");

    display.setCursor(0, 35);
    display.print("Distance: ");

    if (distanceCm < 0) {
        display.println("NO ECHO");
    }
    else {
        display.print(distanceCm);
        display.println(" cm");
    }

    display.display();
}

// =====================================
// Serial Logging
void printMeasurement(const int angleDegrees, const int distanceCm) {
    Serial.print("Angle: ");
    Serial.print(angleDegrees);

    Serial.print(" deg | Distance: ");

    if (distanceCm < 0) {
        Serial.println("NO ECHO");
    }
    else {
        Serial.print(distanceCm);
        Serial.println(" cm");
    }
}

// =====================================
// Radar Sweep
void scanAtAngle(const int angleDegrees) {
    radarServo.write(angleDegrees);

    // Allow the sensor mount to settle before measuring.
    delay(SERVO_SETTLE_MS);

    const int distanceCm = readDistanceCm();

    printMeasurement(
        angleDegrees,
        distanceCm
    );

    updateDisplay(
        angleDegrees,
        distanceCm
    );

    updateLed(distanceCm);
    updateBuzzer(distanceCm);
}


void sweepRadar(const int startAngle, const int endAngle, const int step) {
    if (step > 0) {
        for (int angle = startAngle; angle <= endAngle; angle += step) {
            scanAtAngle(angle);
        }
    }
    else {
        for (int angle = startAngle; angle >= endAngle; angle += step) {
            scanAtAngle(angle);
        }
    }
}

// =====================================
// Initialization
void setup() {
    Serial.begin(115200);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(TRIG_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    )) {
        Serial.println(
            "ERROR: OLED initialization failed."
        );

        while (true) {
            delay(1000);
        }
    }

    display.clearDisplay();
    display.display();

    radarServo.setPeriodHertz(50);

    radarServo.attach(
        SERVO_PIN,
        SERVO_MIN_PULSE_US,
        SERVO_MAX_PULSE_US
    );

    radarServo.write(90);

    delay(1000);

    Serial.println("Radar initialized.");
}

// =====================================
// Main Loop
void loop() {
    sweepRadar(
        SERVO_MIN_ANGLE,
        SERVO_MAX_ANGLE,
        SERVO_STEP
    );

    sweepRadar(
        SERVO_MAX_ANGLE,
        SERVO_MIN_ANGLE,
        -SERVO_STEP
    );
}