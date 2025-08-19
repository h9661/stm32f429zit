/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Header for motor control module (servo and stepper)
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define SERVO_MIN_PULSE_MS      1.0f   // 1ms pulse for 0 degrees
#define SERVO_MAX_PULSE_MS      2.0f   // 2ms pulse for 180 degrees
#define SERVO_PERIOD_MS         20.0f  // 20ms period (50Hz)

#define STEPPER_STEPS_PER_REV   200    // Standard stepper motor steps per revolution
#define STEPPER_MIN_SPEED_RPM   1      // Minimum RPM
#define STEPPER_MAX_SPEED_RPM   100    // Maximum RPM

/* Motor control modes */
typedef enum {
    MOTOR_MODE_IDLE = 0,
    MOTOR_MODE_WAVE,        // Smooth wave motion
    MOTOR_MODE_SWEEP,       // Back and forth sweep
    MOTOR_MODE_STEP,        // Step by step movement
    MOTOR_MODE_RANDOM,      // Random movements
    MOTOR_MODE_DEMO         // Demo all patterns
} MotorMode_t;

/* Servo motor structure */
typedef struct {
    TIM_TypeDef *timer;           // Timer peripheral for PWM
    uint32_t channel;             // Timer channel (1-4)
    float currentAngle;           // Current angle (0-180 degrees)
    float targetAngle;            // Target angle
    float speed;                  // Movement speed (degrees per update)
} ServoMotor_t;

/* Stepper motor structure */
typedef struct {
    GPIO_TypeDef *stepPort;      // GPIO port for step pin
    uint16_t stepPin;             // Step pin
    GPIO_TypeDef *dirPort;       // GPIO port for direction pin
    uint16_t dirPin;              // Direction pin
    uint32_t currentPosition;     // Current position in steps
    uint32_t targetPosition;      // Target position in steps
    uint32_t stepDelay;           // Delay between steps (microseconds)
    uint8_t direction;            // Current direction (0=CW, 1=CCW)
} StepperMotor_t;

/* Public function prototypes ------------------------------------------------*/
void Motor_Control_Init(void);
void Motor_Control_SetMode(MotorMode_t mode);
void Motor_Control_Update(void);

/* Servo motor control functions */
void Servo_Init(ServoMotor_t *servo, TIM_TypeDef *timer, uint32_t channel);
void Servo_SetAngle(ServoMotor_t *servo, float angle);
void Servo_SetSpeed(ServoMotor_t *servo, float speed);
void Servo_Update(ServoMotor_t *servo);

/* Stepper motor control functions */
void Stepper_Init(StepperMotor_t *stepper, GPIO_TypeDef *stepPort, uint16_t stepPin,
                  GPIO_TypeDef *dirPort, uint16_t dirPin);
void Stepper_SetPosition(StepperMotor_t *stepper, uint32_t position);
void Stepper_SetSpeed(StepperMotor_t *stepper, uint16_t rpm);
void Stepper_Step(StepperMotor_t *stepper);
void Stepper_Update(StepperMotor_t *stepper);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */