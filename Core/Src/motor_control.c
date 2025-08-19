/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation (servo and stepper)
  ******************************************************************************
  */

#include "motor_control.h"
#include <math.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
static MotorMode_t currentMode = MOTOR_MODE_IDLE;
static uint32_t modeTimer = 0;
static uint32_t updateCounter = 0;

static ServoMotor_t servo1;
static StepperMotor_t stepper1;

/* Private function prototypes -----------------------------------------------*/
static void Motor_UpdateWaveMode(void);
static void Motor_UpdateSweepMode(void);
static void Motor_UpdateStepMode(void);
static void Motor_UpdateRandomMode(void);
static void Motor_UpdateDemoMode(void);
static float Motor_MapAngle(float value, float in_min, float in_max, float out_min, float out_max);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize motor control system
  * @retval None
  */
void Motor_Control_Init(void)
{
    // Initialize servo on TIM3 Channel 1 (PA6)
    Servo_Init(&servo1, TIM3, 1);
    
    // Initialize stepper on PB8 (step) and PB9 (direction)
    Stepper_Init(&stepper1, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
    
    // Set initial mode
    currentMode = MOTOR_MODE_WAVE;
    modeTimer = 0;
    updateCounter = 0;
}

/**
  * @brief  Set motor control mode
  * @param  mode: New motor mode
  * @retval None
  */
void Motor_Control_SetMode(MotorMode_t mode)
{
    currentMode = mode;
    modeTimer = 0;
    
    // Reset positions when changing modes
    Servo_SetAngle(&servo1, 90.0f);  // Center position
    Stepper_SetPosition(&stepper1, 0);
}

/**
  * @brief  Update motor control (call from main loop or timer)
  * @retval None
  */
void Motor_Control_Update(void)
{
    updateCounter++;
    modeTimer++;
    
    switch (currentMode) {
        case MOTOR_MODE_WAVE:
            Motor_UpdateWaveMode();
            break;
            
        case MOTOR_MODE_SWEEP:
            Motor_UpdateSweepMode();
            break;
            
        case MOTOR_MODE_STEP:
            Motor_UpdateStepMode();
            break;
            
        case MOTOR_MODE_RANDOM:
            Motor_UpdateRandomMode();
            break;
            
        case MOTOR_MODE_DEMO:
            Motor_UpdateDemoMode();
            break;
            
        case MOTOR_MODE_IDLE:
        default:
            // Do nothing
            break;
    }
    
    // Update motors
    Servo_Update(&servo1);
    Stepper_Update(&stepper1);
}

/* Servo motor functions -----------------------------------------------------*/

/**
  * @brief  Initialize servo motor
  * @param  servo: Servo motor structure
  * @param  timer: Timer peripheral
  * @param  channel: Timer channel (1-4)
  * @retval None
  */
void Servo_Init(ServoMotor_t *servo, TIM_TypeDef *timer, uint32_t channel)
{
    servo->timer = timer;
    servo->channel = channel;
    servo->currentAngle = 90.0f;  // Start at center
    servo->targetAngle = 90.0f;
    servo->speed = 1.0f;  // 1 degree per update
    
    // Set initial position
    Servo_SetAngle(servo, 90.0f);
}

/**
  * @brief  Set servo angle
  * @param  servo: Servo motor structure
  * @param  angle: Target angle (0-180 degrees)
  * @retval None
  */
void Servo_SetAngle(ServoMotor_t *servo, float angle)
{
    // Clamp angle to valid range
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    servo->targetAngle = angle;
    
    // Calculate pulse width (1ms to 2ms)
    float pulseWidth = SERVO_MIN_PULSE_MS + (angle / 180.0f) * (SERVO_MAX_PULSE_MS - SERVO_MIN_PULSE_MS);
    
    // Calculate compare value based on timer settings
    // Assuming timer is configured for 1MHz (1us per tick) and period of 20000 (20ms)
    uint32_t compareValue = (uint32_t)(pulseWidth * 1000.0f);  // Convert ms to us
    
    // Set PWM duty cycle using direct register access
    switch (servo->channel) {
        case 1:
            servo->timer->CCR1 = compareValue;
            break;
        case 2:
            servo->timer->CCR2 = compareValue;
            break;
        case 3:
            servo->timer->CCR3 = compareValue;
            break;
        case 4:
            servo->timer->CCR4 = compareValue;
            break;
    }
    
    servo->currentAngle = angle;
}

/**
  * @brief  Set servo movement speed
  * @param  servo: Servo motor structure
  * @param  speed: Movement speed (degrees per update)
  * @retval None
  */
void Servo_SetSpeed(ServoMotor_t *servo, float speed)
{
    servo->speed = speed;
}

/**
  * @brief  Update servo position (smooth movement)
  * @param  servo: Servo motor structure
  * @retval None
  */
void Servo_Update(ServoMotor_t *servo)
{
    if (servo->currentAngle != servo->targetAngle) {
        float diff = servo->targetAngle - servo->currentAngle;
        
        if (fabs(diff) <= servo->speed) {
            servo->currentAngle = servo->targetAngle;
        } else {
            if (diff > 0) {
                servo->currentAngle += servo->speed;
            } else {
                servo->currentAngle -= servo->speed;
            }
        }
        
        Servo_SetAngle(servo, servo->currentAngle);
    }
}

/* Stepper motor functions ---------------------------------------------------*/

/**
  * @brief  Initialize stepper motor
  * @param  stepper: Stepper motor structure
  * @param  stepPort: GPIO port for step pin
  * @param  stepPin: Step pin
  * @param  dirPort: GPIO port for direction pin
  * @param  dirPin: Direction pin
  * @retval None
  */
void Stepper_Init(StepperMotor_t *stepper, GPIO_TypeDef *stepPort, uint16_t stepPin,
                  GPIO_TypeDef *dirPort, uint16_t dirPin)
{
    stepper->stepPort = stepPort;
    stepper->stepPin = stepPin;
    stepper->dirPort = dirPort;
    stepper->dirPin = dirPin;
    stepper->currentPosition = 0;
    stepper->targetPosition = 0;
    stepper->stepDelay = 1000;  // Default 1ms between steps
    stepper->direction = 0;  // Clockwise
    
    // Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks (already enabled in main)
    
    // Configure step and direction pins as outputs
    GPIO_InitStruct.Pin = stepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(stepPort, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = dirPin;
    HAL_GPIO_Init(dirPort, &GPIO_InitStruct);
    
    // Set initial states
    HAL_GPIO_WritePin(stepPort, stepPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);
}

/**
  * @brief  Set stepper target position
  * @param  stepper: Stepper motor structure
  * @param  position: Target position in steps
  * @retval None
  */
void Stepper_SetPosition(StepperMotor_t *stepper, uint32_t position)
{
    stepper->targetPosition = position;
    
    // Update direction
    if (position > stepper->currentPosition) {
        stepper->direction = 0;  // Clockwise
        HAL_GPIO_WritePin(stepper->dirPort, stepper->dirPin, GPIO_PIN_RESET);
    } else {
        stepper->direction = 1;  // Counter-clockwise
        HAL_GPIO_WritePin(stepper->dirPort, stepper->dirPin, GPIO_PIN_SET);
    }
}

/**
  * @brief  Set stepper speed
  * @param  stepper: Stepper motor structure
  * @param  rpm: Speed in RPM
  * @retval None
  */
void Stepper_SetSpeed(StepperMotor_t *stepper, uint16_t rpm)
{
    if (rpm < STEPPER_MIN_SPEED_RPM) rpm = STEPPER_MIN_SPEED_RPM;
    if (rpm > STEPPER_MAX_SPEED_RPM) rpm = STEPPER_MAX_SPEED_RPM;
    
    // Calculate step delay in microseconds
    // Steps per minute = rpm * steps_per_rev
    // Steps per second = (rpm * steps_per_rev) / 60
    // Delay between steps = 1 / steps_per_second
    uint32_t stepsPerSecond = (rpm * STEPPER_STEPS_PER_REV) / 60;
    stepper->stepDelay = 1000000 / stepsPerSecond;  // in microseconds
}

/**
  * @brief  Perform one step
  * @param  stepper: Stepper motor structure
  * @retval None
  */
void Stepper_Step(StepperMotor_t *stepper)
{
    // Generate step pulse
    HAL_GPIO_WritePin(stepper->stepPort, stepper->stepPin, GPIO_PIN_SET);
    // Pulse width (minimum 1us for most drivers)
    for (volatile uint32_t i = 0; i < 10; i++);  // Simple delay
    HAL_GPIO_WritePin(stepper->stepPort, stepper->stepPin, GPIO_PIN_RESET);
    
    // Update position
    if (stepper->direction == 0) {
        stepper->currentPosition++;
    } else {
        if (stepper->currentPosition > 0) {
            stepper->currentPosition--;
        }
    }
}

/**
  * @brief  Update stepper position
  * @param  stepper: Stepper motor structure
  * @retval None
  */
void Stepper_Update(StepperMotor_t *stepper)
{
    static uint32_t lastStepTime = 0;
    
    if (stepper->currentPosition != stepper->targetPosition) {
        // Check if enough time has passed for next step
        if ((HAL_GetTick() * 1000 - lastStepTime) >= stepper->stepDelay) {
            Stepper_Step(stepper);
            lastStepTime = HAL_GetTick() * 1000;
        }
    }
}

/* Private mode update functions ---------------------------------------------*/

/**
  * @brief  Update wave mode (smooth sinusoidal motion)
  * @retval None
  */
static void Motor_UpdateWaveMode(void)
{
    // Create smooth wave motion for servo
    float angle = 90.0f + 60.0f * sinf(updateCounter * 0.05f);
    servo1.targetAngle = angle;
    
    // Create stepping motion for stepper
    uint32_t stepPosition = (uint32_t)(100.0f * (1.0f + sinf(updateCounter * 0.03f)));
    stepper1.targetPosition = stepPosition;
}

/**
  * @brief  Update sweep mode (back and forth motion)
  * @retval None
  */
static void Motor_UpdateSweepMode(void)
{
    static uint8_t direction = 0;
    static float servoAngle = 90.0f;
    static uint32_t stepperPos = 100;
    
    // Servo sweep
    if (direction == 0) {
        servoAngle += 2.0f;
        if (servoAngle >= 150.0f) {
            direction = 1;
        }
    } else {
        servoAngle -= 2.0f;
        if (servoAngle <= 30.0f) {
            direction = 0;
        }
    }
    servo1.targetAngle = servoAngle;
    
    // Stepper sweep
    if (direction == 0) {
        stepperPos += 2;
        if (stepperPos >= 200) {
            stepperPos = 200;
        }
    } else {
        stepperPos -= 2;
        if (stepperPos <= 0) {
            stepperPos = 0;
        }
    }
    stepper1.targetPosition = stepperPos;
}

/**
  * @brief  Update step mode (discrete step movements)
  * @retval None
  */
static void Motor_UpdateStepMode(void)
{
    // Change position every 100 updates
    if (updateCounter % 100 == 0) {
        static uint8_t stepIndex = 0;
        float angles[] = {0, 45, 90, 135, 180};
        uint32_t positions[] = {0, 50, 100, 150, 200};
        
        servo1.targetAngle = angles[stepIndex];
        stepper1.targetPosition = positions[stepIndex];
        
        stepIndex = (stepIndex + 1) % 5;
    }
}

/**
  * @brief  Update random mode (random movements)
  * @retval None
  */
static void Motor_UpdateRandomMode(void)
{
    // Change to random position every 50 updates
    if (updateCounter % 50 == 0) {
        servo1.targetAngle = (float)(rand() % 180);
        stepper1.targetPosition = rand() % 200;
    }
}

/**
  * @brief  Update demo mode (cycle through all modes)
  * @retval None
  */
static void Motor_UpdateDemoMode(void)
{
    // Switch mode every 500 updates
    if (modeTimer >= 500) {
        static uint8_t demoIndex = 0;
        MotorMode_t modes[] = {MOTOR_MODE_WAVE, MOTOR_MODE_SWEEP, 
                               MOTOR_MODE_STEP, MOTOR_MODE_RANDOM};
        
        currentMode = modes[demoIndex];
        demoIndex = (demoIndex + 1) % 4;
        modeTimer = 0;
    }
    
    // Execute current demo mode
    switch (currentMode) {
        case MOTOR_MODE_WAVE:
            Motor_UpdateWaveMode();
            break;
        case MOTOR_MODE_SWEEP:
            Motor_UpdateSweepMode();
            break;
        case MOTOR_MODE_STEP:
            Motor_UpdateStepMode();
            break;
        case MOTOR_MODE_RANDOM:
            Motor_UpdateRandomMode();
            break;
        default:
            break;
    }
}

/**
  * @brief  Map value from one range to another
  * @param  value: Input value
  * @param  in_min: Input minimum
  * @param  in_max: Input maximum
  * @param  out_min: Output minimum
  * @param  out_max: Output maximum
  * @retval Mapped value
  */
static float Motor_MapAngle(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}