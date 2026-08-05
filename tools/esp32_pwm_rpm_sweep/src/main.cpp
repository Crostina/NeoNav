#include <Arduino.h>

constexpr uint8_t LEFT_ENC_A_PIN = 5;
constexpr uint8_t LEFT_ENC_B_PIN = 18;
constexpr uint8_t RIGHT_ENC_A_PIN = 21;
constexpr uint8_t RIGHT_ENC_B_PIN = 19;

constexpr uint8_t LEFT_PWM_PIN = 25;
constexpr uint8_t LEFT_IN1_PIN = 14;
constexpr uint8_t LEFT_IN2_PIN = 13;
constexpr uint8_t RIGHT_PWM_PIN = 26;
constexpr uint8_t RIGHT_IN3_PIN = 33;
constexpr uint8_t RIGHT_IN4_PIN = 27;

constexpr float COUNTS_PER_REV = 1400.0f;
constexpr uint32_t PHASE_MS = 1700;
constexpr uint32_t SETTLE_MS = 600;
constexpr uint32_t SAMPLE_MS = 1000;
constexpr uint32_t STOP_MS = 650;

volatile int32_t leftCountValue = 0;
volatile int32_t rightCountValue = 0;
volatile int8_t leftLastState = 0;
volatile int8_t rightLastState = 0;

int8_t IRAM_ATTR readEncoderState(uint8_t aPin, uint8_t bPin)
{
    const int a = digitalRead(aPin);
    const int b = digitalRead(bPin);
    return static_cast<int8_t>((a << 1) | b);
}

int8_t IRAM_ATTR quadratureDelta(int8_t previous, int8_t current)
{
    const int8_t transition = static_cast<int8_t>((previous << 2) | current);
    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            return 1;
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            return -1;
        default:
            return 0;
    }
}

void IRAM_ATTR handleLeftEncoder()
{
    const int8_t state = readEncoderState(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
    leftCountValue += quadratureDelta(leftLastState, state);
    leftLastState = state;
}

void IRAM_ATTR handleRightEncoder()
{
    const int8_t state = readEncoderState(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
    rightCountValue += quadratureDelta(rightLastState, state);
    rightLastState = state;
}

int32_t leftCount()
{
    noInterrupts();
    const int32_t value = leftCountValue;
    interrupts();
    return value;
}

int32_t rightCount()
{
    noInterrupts();
    const int32_t value = rightCountValue;
    interrupts();
    return value;
}

void setMotor(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int pwm)
{
    pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, pwm);
    } else if (pwm < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, -pwm);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, 0);
    }
}

void stopMotors()
{
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);
    digitalWrite(LEFT_PWM_PIN, LOW);
    digitalWrite(LEFT_IN1_PIN, LOW);
    digitalWrite(LEFT_IN2_PIN, LOW);
    digitalWrite(RIGHT_PWM_PIN, LOW);
    digitalWrite(RIGHT_IN3_PIN, LOW);
    digitalWrite(RIGHT_IN4_PIN, LOW);
}

void runSweepPoint(const char* motor, int pwm)
{
    stopMotors();
    delay(STOP_MS);

    const int32_t leftBefore = leftCount();
    const int32_t rightBefore = rightCount();
    const uint32_t startMs = millis();

    if (motor[0] == 'L') {
        setMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, pwm);
    } else {
        setMotor(RIGHT_PWM_PIN, RIGHT_IN3_PIN, RIGHT_IN4_PIN, pwm);
    }

    delay(SETTLE_MS);
    const int32_t leftSampleStart = leftCount();
    const int32_t rightSampleStart = rightCount();
    const uint32_t sampleStartMs = millis();

    delay(SAMPLE_MS);
    const int32_t leftSampleEnd = leftCount();
    const int32_t rightSampleEnd = rightCount();
    const uint32_t sampleEndMs = millis();

    stopMotors();
    delay(STOP_MS);
    const int32_t leftAfter = leftCount();
    const int32_t rightAfter = rightCount();

    const int32_t sampleTicks =
        motor[0] == 'L' ? (leftSampleEnd - leftSampleStart) : (rightSampleEnd - rightSampleStart);
    const float sampleSeconds = (sampleEndMs - sampleStartMs) / 1000.0f;
    const float cps = sampleTicks / sampleSeconds;
    const float rpm = (cps * 60.0f) / COUNTS_PER_REV;

    Serial.printf(
        "CSV,%s,%d,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%.3f,%.3f\n",
        motor,
        pwm,
        static_cast<unsigned long>(startMs),
        static_cast<long>(leftBefore),
        static_cast<long>(rightBefore),
        static_cast<long>(leftAfter),
        static_cast<long>(rightAfter),
        static_cast<long>(leftAfter - leftBefore),
        static_cast<long>(rightAfter - rightBefore),
        cps,
        rpm);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(LEFT_PWM_PIN, OUTPUT);
    pinMode(LEFT_IN1_PIN, OUTPUT);
    pinMode(LEFT_IN2_PIN, OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_IN3_PIN, OUTPUT);
    pinMode(RIGHT_IN4_PIN, OUTPUT);
    stopMotors();

    pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

    leftLastState = readEncoderState(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
    rightLastState = readEncoderState(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), handleRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), handleRightEncoder, CHANGE);

    Serial.println("EUROBOOT PWM/RPM sweep");
    Serial.println("header,motor,pwm,start_ms,left_before,right_before,left_after,right_after,left_delta,right_delta,cps,rpm");

    const int pwms[] = {40, 50, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200, 220};
    for (int pwm : pwms) {
        runSweepPoint("LEFT_FWD", pwm);
    }
    for (int pwm : pwms) {
        runSweepPoint("LEFT_REV", -pwm);
    }
    for (int pwm : pwms) {
        runSweepPoint("RIGHT_FWD", pwm);
    }
    for (int pwm : pwms) {
        runSweepPoint("RIGHT_REV", -pwm);
    }

    stopMotors();
    Serial.println("SWEEP_DONE");
}

void loop()
{
    stopMotors();
    delay(1000);
}
