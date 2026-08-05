// Euroboot ESP32 + L293D configuration.
// Motor orientation follows Linorobot2: MOTOR1 = left, MOTOR2 = right.

#ifndef EUROBOOT_ESP32_CONFIG_H
#define EUROBOOT_ESP32_CONFIG_H

#define LED_PIN 2

#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_2_IN_MOTOR_DRIVER

// Phase 1 is encoder/wheel control only. With no IMU selected, Linorobot2 uses
// its FakeIMU/FakeMAG classes and still publishes encoder odometry.

#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.001, 0.001, 0.001 }
#define ORI_COV { 0.01, 0.01, 0.01 }
#define MAG_COV { 1e-12, 1e-12, 1e-12 }
#define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

// Starting values only. Linorobot2 PID is discrete per control tick and does
// not include our previous feedforward table yet, so these must be re-tuned.
#define K_P 0.60
#define K_I 0.05
#define K_D 0.00

// Phase 1 feedforward, fitted from the 2026-08-04 off-ground PWM/RPM sweep.
// Formula: pwm = PID(error) + sign(target_rpm) * (offset + slope * abs(target_rpm)).
// Right-forward needs the largest assist at low RPM; left is intentionally
// kept near pure PI because it spins very easily off-ground.
#define USE_EUROBOOT_MOTOR_FEEDFORWARD
#define MOTOR1_FWD_FF_OFFSET 0.0
#define MOTOR1_FWD_FF_SLOPE 0.00
#define MOTOR1_REV_FF_OFFSET 0.0
#define MOTOR1_REV_FF_SLOPE 0.00
#define MOTOR2_FWD_FF_OFFSET 30.0
#define MOTOR2_FWD_FF_SLOPE 0.25
#define MOTOR2_REV_FF_OFFSET 6.0
#define MOTOR2_REV_FF_SLOPE 0.16

#define MOTOR_MAX_RPM 450
#define MAX_RPM_RATIO 0.70
#define MOTOR_OPERATING_VOLTAGE 6
#define MOTOR_POWER_MAX_VOLTAGE 6
#define MOTOR_POWER_MEASURED_VOLTAGE 6

#define COUNTS_PER_REV1 1400
#define COUNTS_PER_REV2 1400
#define COUNTS_PER_REV3 1400
#define COUNTS_PER_REV4 1400

#define WHEEL_DIAMETER 0.04586
#define LR_WHEELS_DISTANCE 0.15216
#define PWM_BITS 8
#define PWM_FREQUENCY 20000
#define USE_COAST_BRAKE_LOW

#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

// Encoders
#define MOTOR1_ENCODER_A 5
#define MOTOR1_ENCODER_B 18

#define MOTOR2_ENCODER_A 21
#define MOTOR2_ENCODER_B 19

#define MOTOR3_ENCODER_A -1
#define MOTOR3_ENCODER_B -1

#define MOTOR4_ENCODER_A -1
#define MOTOR4_ENCODER_B -1

// L293D pins: EN/PWM + two direction inputs per motor.
#define MOTOR1_PWM 25
#define MOTOR1_IN_A 14
#define MOTOR1_IN_B 13

#define MOTOR2_PWM 26
#define MOTOR2_IN_A 33
#define MOTOR2_IN_B 27

#define MOTOR3_PWM -1
#define MOTOR3_IN_A -1
#define MOTOR3_IN_B -1

#define MOTOR4_PWM -1
#define MOTOR4_IN_A -1
#define MOTOR4_IN_B -1

#define PWM_MAX ((1 << PWM_BITS) - 1)
#define PWM_MIN (-PWM_MAX)

#define BAUDRATE 921600
#define NODE_NAME "euroboot_base_node"

#define BOARD_INIT { Wire.begin(); }

#endif
