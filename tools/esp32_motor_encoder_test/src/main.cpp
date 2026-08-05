#include <Arduino.h>
#include <ESP32Encoder.h>

constexpr uint8_t LEFT_PWM_PIN = 25;
constexpr uint8_t LEFT_IN_A_PIN = 14;
constexpr uint8_t LEFT_IN_B_PIN = 13;

constexpr uint8_t RIGHT_PWM_PIN = 26;
constexpr uint8_t RIGHT_IN_A_PIN = 33;
constexpr uint8_t RIGHT_IN_B_PIN = 27;

constexpr uint8_t LEFT_ENC_A_PIN = 5;
constexpr uint8_t LEFT_ENC_B_PIN = 18;
constexpr uint8_t RIGHT_ENC_A_PIN = 21;
constexpr uint8_t RIGHT_ENC_B_PIN = 19;

constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr int TEST_PWM = 80;
constexpr uint32_t REPORT_INTERVAL_MS = 200;

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

struct MotorPins {
    uint8_t pwm;
    uint8_t inA;
    uint8_t inB;
    uint8_t channel;
};

const MotorPins leftMotor{LEFT_PWM_PIN, LEFT_IN_A_PIN, LEFT_IN_B_PIN, LEFT_PWM_CHANNEL};
const MotorPins rightMotor{RIGHT_PWM_PIN, RIGHT_IN_A_PIN, RIGHT_IN_B_PIN, RIGHT_PWM_CHANNEL};

int64_t lastLeftCount = 0;
int64_t lastRightCount = 0;
uint32_t lastReportMs = 0;

void setMotor(const MotorPins& motor, int pwm)
{
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0) {
        digitalWrite(motor.inA, HIGH);
        digitalWrite(motor.inB, LOW);
        ledcWrite(motor.channel, pwm);
    } else if (pwm < 0) {
        digitalWrite(motor.inA, LOW);
        digitalWrite(motor.inB, HIGH);
        ledcWrite(motor.channel, -pwm);
    } else {
        ledcWrite(motor.channel, 0);
        digitalWrite(motor.inA, LOW);
        digitalWrite(motor.inB, LOW);
    }
}

void stopMotors()
{
    setMotor(leftMotor, 0);
    setMotor(rightMotor, 0);
}

void printCounts(const char* phase)
{
    const uint32_t now = millis();
    const uint32_t dtMs = now - lastReportMs;
    if (dtMs < REPORT_INTERVAL_MS) {
        return;
    }

    const int64_t leftCount = leftEncoder.getCount();
    const int64_t rightCount = rightEncoder.getCount();
    const float leftCps = (leftCount - lastLeftCount) * 1000.0f / dtMs;
    const float rightCps = (rightCount - lastRightCount) * 1000.0f / dtMs;

    Serial.printf(
        "t=%lu phase=%s left=%lld lcps=%.1f right=%lld rcps=%.1f\n",
        static_cast<unsigned long>(now),
        phase,
        leftCount,
        leftCps,
        rightCount,
        rightCps);

    lastLeftCount = leftCount;
    lastRightCount = rightCount;
    lastReportMs = now;
}

void runPhase(const char* name, int leftPwm, int rightPwm, uint32_t durationMs)
{
    Serial.printf("\nPHASE %s left_pwm=%d right_pwm=%d duration_ms=%lu\n",
                  name,
                  leftPwm,
                  rightPwm,
                  static_cast<unsigned long>(durationMs));

    setMotor(leftMotor, leftPwm);
    setMotor(rightMotor, rightPwm);

    const uint32_t startMs = millis();
    lastReportMs = startMs;
    lastLeftCount = leftEncoder.getCount();
    lastRightCount = rightEncoder.getCount();

    while (millis() - startMs < durationMs) {
        printCounts(name);
        delay(10);
    }

    stopMotors();
    delay(300);
    printCounts("stop_after_phase");
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(leftMotor.inA, OUTPUT);
    pinMode(leftMotor.inB, OUTPUT);
    pinMode(rightMotor.inA, OUTPUT);
    pinMode(rightMotor.inB, OUTPUT);

    ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
    ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);

    stopMotors();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    leftEncoder.attachFullQuad(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
    rightEncoder.attachFullQuad(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
    leftEncoder.clearCount();
    rightEncoder.clearCount();

    Serial.println("\nEUROBOOT ESP32 raw motor + encoder test");
    Serial.println("Left: PWM=25 IN_A=14 IN_B=13 ENC_A=5 ENC_B=18");
    Serial.println("Right: PWM=26 IN_A=33 IN_B=27 ENC_A=21 ENC_B=19");
    Serial.println("Sequence: idle, left forward/reverse, right forward/reverse, both forward, stop.");
    Serial.println("Counts sign tells us encoder direction. CPS tells us whether the encoder is alive.");

    runPhase("idle", 0, 0, 2000);
    runPhase("left_forward", TEST_PWM, 0, 2500);
    runPhase("left_reverse", -TEST_PWM, 0, 2500);
    runPhase("right_forward", 0, TEST_PWM, 2500);
    runPhase("right_reverse", 0, -TEST_PWM, 2500);
    runPhase("both_forward", TEST_PWM, TEST_PWM, 2500);
    stopMotors();
    Serial.println("\nTEST_DONE motors stopped");
}

void loop()
{
    stopMotors();
    printCounts("done_idle");
    delay(50);
}

