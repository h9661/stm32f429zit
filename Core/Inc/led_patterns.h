/**
  ******************************************************************************
  * @file           : led_patterns.h
  * @brief          : Beautiful LED pattern show system
  ******************************************************************************
  */

#ifndef __LED_PATTERNS_H
#define __LED_PATTERNS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* LED Pattern Types */
typedef enum {
    PATTERN_WAVE,           // Smooth wave effect
    PATTERN_PULSE,          // Breathing/pulse effect
    PATTERN_CHASE,          // Chase light effect
    PATTERN_RAINBOW,        // Rainbow color transition
    PATTERN_SPARKLE,        // Random sparkle effect
    PATTERN_THEATER,        // Theater marquee effect
    PATTERN_FIRE,           // Fire/flame effect
    PATTERN_COMET,          // Comet tail effect
    PATTERN_HEARTBEAT,      // Heartbeat rhythm
    PATTERN_STROBE,         // Strobe light effect
    PATTERN_COUNT
} PatternType_t;

/* Pattern Configuration */
typedef struct {
    PatternType_t type;
    uint32_t speed;         // Speed in ms (lower = faster)
    uint8_t intensity;      // Intensity 0-100%
    uint8_t direction;      // 0 = forward, 1 = reverse
} PatternConfig_t;

/* Pattern State */
typedef struct {
    uint32_t tick;          // Current animation tick
    uint32_t lastUpdate;    // Last update timestamp
    uint8_t phase;          // Current phase of animation
    uint8_t subPhase;       // Sub-phase for complex patterns
    uint16_t position;      // Position in pattern sequence
    uint8_t brightness[3];  // RGB brightness values (0-255)
} PatternState_t;

/* Function Prototypes */
void LED_Pattern_Init(void);
void LED_Pattern_Update(uint32_t currentTick);
void LED_Pattern_SetPattern(PatternType_t pattern);
void LED_Pattern_NextPattern(void);
void LED_Pattern_SetSpeed(uint32_t speed);
void LED_Pattern_SetIntensity(uint8_t intensity);

/* Pattern Implementation Functions */
void Pattern_Wave(PatternState_t* state, PatternConfig_t* config);
void Pattern_Pulse(PatternState_t* state, PatternConfig_t* config);
void Pattern_Chase(PatternState_t* state, PatternConfig_t* config);
void Pattern_Rainbow(PatternState_t* state, PatternConfig_t* config);
void Pattern_Sparkle(PatternState_t* state, PatternConfig_t* config);
void Pattern_Theater(PatternState_t* state, PatternConfig_t* config);
void Pattern_Fire(PatternState_t* state, PatternConfig_t* config);
void Pattern_Comet(PatternState_t* state, PatternConfig_t* config);
void Pattern_Heartbeat(PatternState_t* state, PatternConfig_t* config);
void Pattern_Strobe(PatternState_t* state, PatternConfig_t* config);

/* Utility Functions */
void LED_SetBrightness(uint8_t red, uint8_t green, uint8_t blue);
uint8_t LED_CalculateSineWave(uint16_t position, uint16_t period);
uint8_t LED_Random(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_PATTERNS_H */