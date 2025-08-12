/**
  ******************************************************************************
  * @file           : led_patterns.c
  * @brief          : Beautiful LED pattern show implementation
  ******************************************************************************
  */

#include "led_patterns.h"
#include <stdlib.h>

/* Private variables */
static PatternConfig_t currentConfig;
static PatternState_t patternState;
static uint32_t randomSeed = 0x12345678;

/* Sine lookup table for smooth animations (0-255) */
static const uint8_t sineTable[64] = {
    128, 140, 152, 165, 176, 188, 198, 208,
    218, 226, 234, 240, 245, 250, 253, 254,
    255, 254, 253, 250, 245, 240, 234, 226,
    218, 208, 198, 188, 176, 165, 152, 140,
    128, 115, 103, 90, 79, 67, 57, 47,
    37, 29, 21, 15, 10, 5, 2, 1,
    0, 1, 2, 5, 10, 15, 21, 29,
    37, 47, 57, 67, 79, 90, 103, 115
};

/**
  * @brief  Initialize LED pattern system
  */
void LED_Pattern_Init(void)
{
    currentConfig.type = PATTERN_WAVE;
    currentConfig.speed = 50;
    currentConfig.intensity = 100;
    currentConfig.direction = 0;
    
    patternState.tick = 0;
    patternState.lastUpdate = 0;
    patternState.phase = 0;
    patternState.subPhase = 0;
    patternState.position = 0;
    patternState.brightness[0] = 0;
    patternState.brightness[1] = 0;
    patternState.brightness[2] = 0;
}

/**
  * @brief  Update LED pattern (call from SysTick)
  * @param  currentTick: Current system tick
  */
void LED_Pattern_Update(uint32_t currentTick)
{
    // Check if it's time to update
    if (currentTick - patternState.lastUpdate < currentConfig.speed) {
        return;
    }
    
    patternState.lastUpdate = currentTick;
    patternState.tick++;
    
    // Execute current pattern
    switch (currentConfig.type) {
        case PATTERN_WAVE:
            Pattern_Wave(&patternState, &currentConfig);
            break;
        case PATTERN_PULSE:
            Pattern_Pulse(&patternState, &currentConfig);
            break;
        case PATTERN_CHASE:
            Pattern_Chase(&patternState, &currentConfig);
            break;
        case PATTERN_RAINBOW:
            Pattern_Rainbow(&patternState, &currentConfig);
            break;
        case PATTERN_SPARKLE:
            Pattern_Sparkle(&patternState, &currentConfig);
            break;
        case PATTERN_THEATER:
            Pattern_Theater(&patternState, &currentConfig);
            break;
        case PATTERN_FIRE:
            Pattern_Fire(&patternState, &currentConfig);
            break;
        case PATTERN_COMET:
            Pattern_Comet(&patternState, &currentConfig);
            break;
        case PATTERN_HEARTBEAT:
            Pattern_Heartbeat(&patternState, &currentConfig);
            break;
        case PATTERN_STROBE:
            Pattern_Strobe(&patternState, &currentConfig);
            break;
        default:
            break;
    }
    
    // Apply brightness with intensity scaling
    uint8_t scaledRed = (patternState.brightness[0] * currentConfig.intensity) / 100;
    uint8_t scaledGreen = (patternState.brightness[1] * currentConfig.intensity) / 100;
    uint8_t scaledBlue = (patternState.brightness[2] * currentConfig.intensity) / 100;
    
    LED_SetBrightness(scaledRed, scaledGreen, scaledBlue);
}

/**
  * @brief  Wave pattern - smooth sine wave across LEDs
  */
void Pattern_Wave(PatternState_t* state, PatternConfig_t* config)
{
    // Calculate phase offsets for each LED
    uint16_t redPhase = state->position;
    uint16_t greenPhase = (state->position + 21) % 64;  // 120 degrees
    uint16_t bluePhase = (state->position + 42) % 64;   // 240 degrees
    
    state->brightness[0] = sineTable[redPhase];
    state->brightness[1] = sineTable[greenPhase];
    state->brightness[2] = sineTable[bluePhase];
    
    // Update position
    if (config->direction == 0) {
        state->position = (state->position + 1) % 64;
    } else {
        state->position = (state->position + 63) % 64;
    }
}

/**
  * @brief  Pulse pattern - breathing effect
  */
void Pattern_Pulse(PatternState_t* state, PatternConfig_t* config)
{
    uint8_t brightness = sineTable[state->position];
    
    // All LEDs pulse together
    state->brightness[0] = brightness;
    state->brightness[1] = brightness;
    state->brightness[2] = brightness;
    
    // Update position for smooth breathing
    state->position = (state->position + 1) % 64;
}

/**
  * @brief  Chase pattern - lights chase each other
  */
void Pattern_Chase(PatternState_t* state, PatternConfig_t* config)
{
    // Reset all LEDs
    state->brightness[0] = 0;
    state->brightness[1] = 0;
    state->brightness[2] = 0;
    
    // Light up one LED at a time with trailing effect
    uint8_t activeChannel = state->phase % 3;
    state->brightness[activeChannel] = 255;
    
    // Add trailing glow
    uint8_t prevChannel = (state->phase + 2) % 3;
    state->brightness[prevChannel] = 64;
    
    // Update phase
    state->phase = (state->phase + (config->direction ? 2 : 1)) % 3;
}

/**
  * @brief  Rainbow pattern - smooth color transitions
  */
void Pattern_Rainbow(PatternState_t* state, PatternConfig_t* config)
{
    // Use HSV to RGB conversion logic (simplified)
    uint8_t hue = state->position * 4;  // 0-255 hue range
    
    if (hue < 85) {
        // Red to Green
        state->brightness[0] = 255 - hue * 3;
        state->brightness[1] = hue * 3;
        state->brightness[2] = 0;
    } else if (hue < 170) {
        // Green to Blue
        hue -= 85;
        state->brightness[0] = 0;
        state->brightness[1] = 255 - hue * 3;
        state->brightness[2] = hue * 3;
    } else {
        // Blue to Red
        hue -= 170;
        state->brightness[0] = hue * 3;
        state->brightness[1] = 0;
        state->brightness[2] = 255 - hue * 3;
    }
    
    // Update position
    state->position = (state->position + 1) % 64;
}

/**
  * @brief  Sparkle pattern - random twinkling
  */
void Pattern_Sparkle(PatternState_t* state, PatternConfig_t* config)
{
    // Fade existing LEDs
    for (int i = 0; i < 3; i++) {
        if (state->brightness[i] > 10) {
            state->brightness[i] -= 10;
        } else {
            state->brightness[i] = 0;
        }
    }
    
    // Randomly light up an LED
    if (LED_Random() % 10 < 3) {  // 30% chance
        uint8_t channel = LED_Random() % 3;
        state->brightness[channel] = 200 + (LED_Random() % 56);
    }
}

/**
  * @brief  Theater pattern - marquee lights
  */
void Pattern_Theater(PatternState_t* state, PatternConfig_t* config)
{
    // Theater chase effect
    state->brightness[0] = (state->phase == 0) ? 255 : 0;
    state->brightness[1] = (state->phase == 1) ? 255 : 0;
    state->brightness[2] = (state->phase == 2) ? 255 : 0;
    
    // Update phase
    state->phase = (state->phase + 1) % 3;
    
    // Add some variation every 3 cycles
    if (state->phase == 0) {
        state->subPhase = (state->subPhase + 1) % 3;
        if (state->subPhase == 0) {
            // All on briefly
            state->brightness[0] = state->brightness[1] = state->brightness[2] = 128;
        }
    }
}

/**
  * @brief  Fire pattern - flickering flame effect
  */
void Pattern_Fire(PatternState_t* state, PatternConfig_t* config)
{
    // Fire colors: red dominant, some yellow, minimal blue
    uint8_t flicker = 180 + (LED_Random() % 76);  // 180-255
    
    state->brightness[0] = flicker;  // Red always bright
    state->brightness[1] = flicker / 3;  // Green for yellow tint
    state->brightness[2] = 0;  // No blue in fire
    
    // Occasional bright flare
    if (LED_Random() % 20 == 0) {
        state->brightness[0] = 255;
        state->brightness[1] = 128;
    }
}

/**
  * @brief  Comet pattern - moving light with tail
  */
void Pattern_Comet(PatternState_t* state, PatternConfig_t* config)
{
    // Fade all LEDs
    for (int i = 0; i < 3; i++) {
        if (state->brightness[i] > 20) {
            state->brightness[i] -= 20;
        } else {
            state->brightness[i] = 0;
        }
    }
    
    // Move comet head
    uint8_t head = state->phase % 3;
    state->brightness[head] = 255;
    
    // Update phase
    state->phase = (state->phase + 1) % 3;
}

/**
  * @brief  Heartbeat pattern - double pulse rhythm
  */
void Pattern_Heartbeat(PatternState_t* state, PatternConfig_t* config)
{
    // Heartbeat has two pulses followed by a pause
    uint8_t intensity = 0;
    
    if (state->position < 10) {
        // First beat
        intensity = sineTable[(state->position * 6) % 64];
    } else if (state->position >= 15 && state->position < 25) {
        // Second beat
        intensity = sineTable[((state->position - 15) * 6) % 64];
    }
    
    // Red heartbeat
    state->brightness[0] = intensity;
    state->brightness[1] = 0;
    state->brightness[2] = intensity / 8;  // Slight purple tint
    
    // Update position
    state->position = (state->position + 1) % 50;  // Full cycle
}

/**
  * @brief  Strobe pattern - rapid flashing
  */
void Pattern_Strobe(PatternState_t* state, PatternConfig_t* config)
{
    // Simple on/off strobe
    uint8_t intensity = (state->phase % 2) ? 255 : 0;
    
    state->brightness[0] = intensity;
    state->brightness[1] = intensity;
    state->brightness[2] = intensity;
    
    state->phase = !state->phase;
}

/**
  * @brief  Set LED brightness (software PWM simulation)
  */
void LED_SetBrightness(uint8_t red, uint8_t green, uint8_t blue)
{
    // Software PWM using fast switching
    // Called from SysTick (1ms), so we get ~256Hz PWM frequency
    static uint8_t pwmCounter = 0;
    static uint8_t dither = 0;
    
    // Add dithering for smoother gradients
    dither = (dither + 17) & 0x0F;  // Prime number for better distribution
    
    // Apply dithering to smooth out PWM
    uint8_t redThreshold = red + ((dither < 8) ? 1 : 0);
    uint8_t greenThreshold = green + ((dither < 4) ? 1 : 0);
    uint8_t blueThreshold = blue + ((dither < 12) ? 1 : 0);
    
    // Software PWM comparison
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 
                      (redThreshold > pwmCounter) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 
                      (greenThreshold > pwmCounter) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 
                      (blueThreshold > pwmCounter) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // Increment PWM counter (8-bit for 256 levels)
    pwmCounter++;
}

/**
  * @brief  Set current pattern
  */
void LED_Pattern_SetPattern(PatternType_t pattern)
{
    if (pattern < PATTERN_COUNT) {
        currentConfig.type = pattern;
        // Reset state for new pattern
        patternState.phase = 0;
        patternState.subPhase = 0;
        patternState.position = 0;
    }
}

/**
  * @brief  Switch to next pattern
  */
void LED_Pattern_NextPattern(void)
{
    currentConfig.type = (currentConfig.type + 1) % PATTERN_COUNT;
    // Reset state for new pattern
    patternState.phase = 0;
    patternState.subPhase = 0;
    patternState.position = 0;
}

/**
  * @brief  Set animation speed
  */
void LED_Pattern_SetSpeed(uint32_t speed)
{
    currentConfig.speed = speed;
}

/**
  * @brief  Set pattern intensity
  */
void LED_Pattern_SetIntensity(uint8_t intensity)
{
    if (intensity <= 100) {
        currentConfig.intensity = intensity;
    }
}

/**
  * @brief  Simple pseudo-random number generator
  */
uint8_t LED_Random(void)
{
    randomSeed = randomSeed * 1103515245 + 12345;
    return (randomSeed >> 16) & 0xFF;
}

/**
  * @brief  Calculate sine wave value
  */
uint8_t LED_CalculateSineWave(uint16_t position, uint16_t period)
{
    uint16_t index = (position * 64) / period;
    return sineTable[index % 64];
}