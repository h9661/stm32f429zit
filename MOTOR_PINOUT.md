# Motor Connection Guide for Nucleo STM32F429ZIT6

## Board: Nucleo-144 STM32F429ZIT6

### 🔌 Servo Motor Connections

#### Servo Motor Pin Definition:
- **PWM Signal Pin: PA6** (CN12 connector, Pin 13)
  - Timer: TIM3 Channel 1
  - PWM Frequency: 50Hz (20ms period)
  - Pulse Width: 1-2ms (1ms = 0°, 1.5ms = 90°, 2ms = 180°)

#### Servo Motor Wiring:
| Servo Wire | Nucleo Pin | Location | Description |
|------------|------------|----------|-------------|
| Signal (Orange/White) | PA6 | CN12 Pin 13 | PWM control signal |
| Power (Red) | 5V | CN8 Pin 7 | 5V power supply |
| Ground (Brown/Black) | GND | CN8 Pin 11 | Ground |

**Important**: Most servo motors require 5V power. The Nucleo board provides 5V on CN8 Pin 7.

---

### ⚙️ Stepper Motor Connections

#### Stepper Motor Pin Definition:
- **STEP Pin: PB8** (CN12 connector, Pin 3)
  - Digital output for step pulses
  - Each pulse = one step
  
- **DIR Pin: PB9** (CN12 connector, Pin 5)
  - Digital output for direction control
  - LOW = Clockwise, HIGH = Counter-clockwise

#### Stepper Motor Driver Wiring (A4988/DRV8825):
| Driver Pin | Nucleo Pin | Location | Description |
|------------|------------|----------|-------------|
| STEP | PB8 | CN12 Pin 3 | Step pulse signal |
| DIR | PB9 | CN12 Pin 5 | Direction control |
| VDD | 3.3V | CN8 Pin 16 | Logic power (3.3V) |
| GND | GND | CN8 Pin 11 | Ground |
| VMOT | External 12-24V | - | Motor power supply |
| GND (Motor) | External GND | - | Motor ground |
| ENABLE | GND (optional) | CN8 Pin 11 | Enable driver (active LOW) |

#### Stepper Motor to Driver Connection:
| Motor Wire | Driver Pin | Description |
|------------|------------|-------------|
| A+ (Black) | 1A | Coil A positive |
| A- (Green) | 1B | Coil A negative |
| B+ (Red) | 2A | Coil B positive |
| B- (Blue) | 2B | Coil B negative |

---

## 📍 Physical Pin Locations on Nucleo Board

### CN12 Connector (Arduino Shield Compatible):
```
        CN12 (Digital pins)
    ┌─────────────────────┐
    │ 1  - D15 (PB8) STEP │ ← Stepper STEP
    │ 2  - D14           │
    │ 3  - D13 (PA5)     │
    │ 4  - D12           │
    │ 5  - D11 (PB9) DIR │ ← Stepper DIR
    │ 6  - D10           │
    │ ...                │
    │ 13 - D6 (PA6) PWM  │ ← Servo Signal
    │ ...                │
    └─────────────────────┘
```

### CN8 Power Connector:
```
        CN8 (Power)
    ┌─────────────────────┐
    │ 7  - 5V            │ ← Servo Power
    │ 11 - GND           │ ← Common Ground
    │ 16 - 3.3V          │ ← Driver Logic Power
    └─────────────────────┘
```

---

## ⚡ Power Requirements

### Servo Motor:
- **Voltage**: 4.8-6V (typically 5V)
- **Current**: 100-500mA idle, up to 2A stall
- **Power Source**: Nucleo 5V pin (limited current) or external 5V supply

### Stepper Motor:
- **Logic Voltage**: 3.3V (from Nucleo)
- **Motor Voltage**: 12-24V (external power supply required)
- **Current**: Depends on motor (typically 0.5-2A per phase)
- **Power Source**: External power supply with common ground

---

## 🔧 Quick Setup Checklist

### For Servo Motor:
1. ✅ Connect signal wire to PA6 (CN12 Pin 13)
2. ✅ Connect power wire to 5V (CN8 Pin 7)
3. ✅ Connect ground wire to GND (CN8 Pin 11)

### For Stepper Motor with Driver:
1. ✅ Connect STEP to PB8 (CN12 Pin 3)
2. ✅ Connect DIR to PB9 (CN12 Pin 5)
3. ✅ Connect driver VDD to 3.3V (CN8 Pin 16)
4. ✅ Connect driver GND to board GND (CN8 Pin 11)
5. ✅ Connect external 12-24V power to driver VMOT
6. ✅ Connect motor coils to driver outputs (1A, 1B, 2A, 2B)
7. ✅ Ensure common ground between Nucleo and external power

---

## ⚠️ Safety Notes

1. **Power Isolation**: Use separate power supplies for motors to avoid brownouts
2. **Current Limiting**: Set appropriate current limits on stepper driver
3. **Heat Dissipation**: Add heatsinks to stepper drivers if running continuously
4. **Ground Connection**: Always connect all grounds together (Nucleo + external supplies)
5. **Voltage Levels**: PA6, PB8, PB9 are 3.3V tolerant only - do not apply 5V!

---

## 📝 Software Configuration Summary

### Servo (PA6):
- Timer: TIM3 Channel 1
- Mode: PWM Mode 1
- Frequency: 50Hz (20ms period)
- Duty Cycle: 5-10% (1-2ms pulse)

### Stepper (PB8, PB9):
- PB8: GPIO Output Push-Pull (STEP)
- PB9: GPIO Output Push-Pull (DIR)
- Speed: Controlled by step pulse frequency
- Direction: PB9 HIGH/LOW

---

## 🎮 Control Modes Available

The implemented software supports these motion patterns:
1. **Wave Mode**: Smooth sinusoidal motion
2. **Sweep Mode**: Back and forth movement
3. **Step Mode**: Discrete position changes
4. **Random Mode**: Random position movements
5. **Demo Mode**: Cycles through all patterns

Press the USER button to cycle through modes!