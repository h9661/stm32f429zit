# Button LED Control Feature

## Overview
This feature implements a multi-mode LED control system using the user button on the STM32F429ZI Nucleo board. Different button press patterns trigger different LED modes.

## Button Press Patterns

### 1. Single Press
- **Action**: Press and release the button once
- **Result**: Green LED turns ON
- **Console Output**: "Single press detected! Mode: Green LED only"

### 2. Double Press
- **Action**: Press and release the button twice within 400ms
- **Result**: Green and Blue LEDs turn ON
- **Console Output**: "Double press detected! Mode: Green + Blue LEDs"

### 3. Long Press
- **Action**: Press and hold the button for more than 1 second
- **Result**: All three LEDs (Green, Blue, Red) blink at 2Hz
- **Console Output**: "Long press detected! Mode: All LEDs blinking"

## Technical Implementation

### Hardware Configuration
- **User Button**: PC13 (configured with interrupt on both edges)
- **Green LED**: PB0
- **Blue LED**: PB7
- **Red LED**: PB14

### Software Architecture

#### State Machine
The button handler implements a state machine with the following states:
- `BUTTON_IDLE`: Waiting for button press
- `BUTTON_PRESSED`: Button is currently pressed
- `BUTTON_RELEASED`: Button was just released
- `BUTTON_WAIT_DOUBLE`: Waiting to detect double-click

#### Timing Parameters
- **Debounce Time**: 50ms
- **Double Click Window**: 400ms
- **Long Press Threshold**: 1000ms
- **LED Blink Rate**: 250ms (2Hz)

#### Key Functions
1. `HAL_GPIO_EXTI_Callback()`: Interrupt handler for button events
2. `ProcessButtonPress()`: State machine processor
3. `UpdateLEDs()`: LED state updater
4. `SetLEDMode()`: Mode transition handler

### Interrupt Handling
The implementation uses EXTI (External Interrupt) on PC13 with:
- Rising and falling edge detection
- Hardware debouncing (50ms software debounce)
- Priority 0 interrupt handling

## Usage Instructions

1. **Build and Flash** the project to your STM32F429ZI Nucleo board
2. **Open a serial terminal** to see debug messages (115200 baud)
3. **Press the blue user button** with different patterns:
   - Quick press → Green LED
   - Double press → Green + Blue LEDs
   - Long press (>1s) → All LEDs blinking

## Code Structure

### Modified Files
- `Core/Inc/main.h`: Added button pin definitions and timing constants
- `Core/Src/main.c`: Implemented button handling logic and LED control
- `Core/Src/stm32f4xx_it.c`: Added EXTI interrupt handler
- `Core/Inc/stm32f4xx_it.h`: Added interrupt handler declaration

### New Features Added
- Button state machine for pattern detection
- LED mode enumeration
- Debounce logic
- Multi-pattern recognition (single, double, long press)
- Console debug output

## Future Enhancements
- Add more LED patterns (e.g., sequential blinking)
- Implement triple-click detection
- Add PWM-based LED brightness control
- Store last mode in flash memory
- Add button hold-and-release patterns