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

constexpr int TEST_PWM = 180;
constexpr uint32_t REPORT_INTERVAL_MS = 100;

volatile int32_t leftCountValue = 0;
volatile int32_t rightCountValue = 0;
volatile int8_t leftLastState = 0;
volatile int8_t rightLastState = 0;

int32_t lastLeftCount = 0;
int32_t lastRightCount = 0;
uint32_t lastReportMs = 0;

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

void printCounts(const char* phase)
{
    const uint32_t now = millis();
    const uint32_t dtMs = now - lastReportMs;
    if (dtMs < REPORT_INTERVAL_MS) {
        return;
    }

    const int32_t left = leftCount();
    const int32_t right = rightCount();
    const float lcps = (left - lastLeftCount) * 1000.0f / dtMs;
    const float rcps = (right - lastRightCount) * 1000.0f / dtMs;

    Serial.printf("t=%lu phase=%s left=%ld lcps=%.1f right=%ld rcps=%.1f pins L(%d,%d,%d) R(%d,%d,%d) encR(%d,%d)\n",
                  static_cast<unsigned long>(now),
                  phase,
                  static_cast<long>(left),
                  lcps,
                  static_cast<long>(right),
                  rcps,
                  digitalRead(LEFT_PWM_PIN),
                  digitalRead(LEFT_IN1_PIN),
                  digitalRead(LEFT_IN2_PIN),
                  digitalRead(RIGHT_PWM_PIN),
                  digitalRead(RIGHT_IN3_PIN),
                  digitalRead(RIGHT_IN4_PIN),
                  digitalRead(RIGHT_ENC_A_PIN),
                  digitalRead(RIGHT_ENC_B_PIN));

    lastLeftCount = left;
    lastRightCount = right;
    lastReportMs = now;
}

void runPhase(const char* name, int leftPwm, int rightPwm, bool swapRightPins, uint32_t durationMs)
{
    stopMotors();
    delay(500);
    Serial.printf("\nPHASE %s left_pwm=%d right_pwm=%d swap_right=%s duration_ms=%lu\n",
                  name,
                  leftPwm,
                  rightPwm,
                  swapRightPins ? "true" : "false",
                  static_cast<unsigned long>(durationMs));

    lastReportMs = millis();
    lastLeftCount = leftCount();
    lastRightCount = rightCount();

    if (leftPwm != 0) {
        setMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, leftPwm);
    }
    if (rightPwm != 0) {
        if (swapRightPins) {
            setMotor(RIGHT_PWM_PIN, RIGHT_IN4_PIN, RIGHT_IN3_PIN, rightPwm);
        } else {
            setMotor(RIGHT_PWM_PIN, RIGHT_IN3_PIN, RIGHT_IN4_PIN, rightPwm);
        }
    }

    const uint32_t startMs = millis();
    while (millis() - startMs < durationMs) {
        printCounts(name);
        delay(10);
    }

    stopMotors();
    delay(500);
    printCounts("stop_after_phase");
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

    Serial.println("\nEUROBOOT motor isolation test, backup-style analogWrite + manual encoder ISR");
    Serial.println("Left: PWM=25 IN1=14 IN2=13 ENC=5/18");
    Serial.println("Right canonical: PWM=26 IN3=33 IN4=27 ENC=21/19");
    Serial.println("Right swapped phase uses PWM=26 IN_A=27 IN_B=33.");

    runPhase("idle", 0, 0, false, 1500);
    runPhase("left_only_forward", TEST_PWM, 0, false, 2000);
    runPhase("left_only_reverse", -TEST_PWM, 0, false, 2000);
    runPhase("right_only_forward_canonical", 0, TEST_PWM, false, 2000);
    runPhase("right_only_reverse_canonical", 0, -TEST_PWM, false, 2000);
    runPhase("right_only_forward_swapped", 0, TEST_PWM, true, 2000);
    runPhase("right_only_reverse_swapped", 0, -TEST_PWM, true, 2000);

    stopMotors();
    Serial.println("\nTEST_DONE motors stopped");
}

void loop()
{
    stopMotors();
    printCounts("done_idle");
    delay(50);
}
