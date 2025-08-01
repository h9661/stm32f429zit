# STM32F429ZIT6 Nucleo Project Index

## Project Overview

This is an STM32F429ZIT6 microcontroller project targeting the Nucleo-144 development board (NUCLEO-F429ZI). The project implements a basic LED blinking application demonstrating GPIO control using the STM32 HAL (Hardware Abstraction Layer) library.

**Current Status**: Basic LED demonstration with three LEDs (Red, Green, Blue) blinking simultaneously with debug output via printf.

## Hardware Details

### Target Microcontroller: STM32F429ZIT6
- **Architecture**: ARM Cortex-M4 with FPU
- **Package**: LQFP144
- **Flash Memory**: 2048KB (2MB)
- **SRAM**: 192KB main + 64KB CCM (Core Coupled Memory)
- **Clock Speed**: Currently running at 16MHz (HSI), capable up to 180MHz
- **Advanced Features**: DSP instructions, floating-point unit, DMA2D graphics accelerator, LCD-TFT controller

### Pin Configuration (Current Implementation)
```
GPIOB_Pin_0  → LED_GREEN  (Nucleo Green LED)
GPIOB_Pin_7  → LED_BLUE   (Nucleo Blue LED)  
GPIOB_Pin_14 → LED_RED    (Nucleo Red LED)
```

### Clock Configuration
- **Primary Clock Source**: HSI (High Speed Internal) - 16MHz
- **System Clock**: 16MHz (not optimized - can be increased to 180MHz)
- **AHB/APB1/APB2**: All running at 16MHz
- **Voltage Scale**: Scale 3 (power-optimized for lower frequencies)

## Software Architecture

### Development Framework
- **IDE**: STM32CubeIDE (Eclipse-based)
- **Code Generation**: STM32CubeMX v6.15.0
- **HAL Version**: STM32Cube FW_F4 V1.28.2
- **Build System**: Eclipse CDT with GCC toolchain
- **Debug Interface**: ST-Link v2 with SWD protocol

### Enabled HAL Modules
```c
HAL_GPIO_MODULE_ENABLED     // GPIO control
HAL_EXTI_MODULE_ENABLED     // External interrupts  
HAL_DMA_MODULE_ENABLED      // Direct Memory Access
HAL_RCC_MODULE_ENABLED      // Reset and Clock Control
HAL_FLASH_MODULE_ENABLED    // Flash memory control
HAL_PWR_MODULE_ENABLED      // Power management
HAL_CORTEX_MODULE_ENABLED   // Cortex-M4 specific features
```

### Application Flow
1. **System Initialization**: HAL_Init() → SystemClock_Config() → GPIO_Init()
2. **Main Loop**: Infinite loop toggling all three LEDs every 1000ms with printf debug output
3. **Debug Output**: Console logging via printf (likely redirected through SWO or UART)

## File Structure Documentation

```
nucleo/
├── Core/                           # Application source code
│   ├── Inc/                       # Header files
│   │   ├── main.h                 # Main application header
│   │   ├── stm32f4xx_hal_conf.h   # HAL configuration
│   │   └── stm32f4xx_it.h         # Interrupt handlers header
│   ├── Src/                       # Source files
│   │   ├── main.c                 # Main application (LED blinking)
│   │   ├── stm32f4xx_hal_msp.c    # HAL MSP initialization
│   │   ├── stm32f4xx_it.c         # Interrupt service routines
│   │   ├── syscalls.c             # System calls (printf support)
│   │   ├── sysmem.c               # Memory management
│   │   └── system_stm32f4xx.c     # System initialization
│   └── Startup/
│       └── startup_stm32f429zitx.s # Startup assembly code
├── Drivers/                        # STM32 HAL and CMSIS drivers
│   ├── STM32F4xx_HAL_Driver/      # HAL peripheral drivers
│   └── CMSIS/                     # Cortex Microcontroller Software Interface
├── Debug/                         # Build output directory
│   ├── nucleo.elf                 # Executable binary
│   ├── nucleo.map                 # Memory map
│   └── [object files]             # Compiled object files
├── projects/                      # Empty - reserved for future projects
├── nucleo.ioc                     # STM32CubeMX project file
├── STM32F429ZITX_FLASH.ld         # Flash memory linker script
├── STM32F429ZITX_RAM.ld           # RAM linker script  
├── .project                       # Eclipse project configuration
├── .cproject                      # Eclipse C/C++ configuration
├── nucleo Debug.launch            # Debug configuration
└── STM32F429ZIT6_Firmware_Roadmap.md # Comprehensive learning roadmap
```

## Build Instructions

### Prerequisites
1. **STM32CubeIDE** installed with STM32F4 package
2. **ST-Link drivers** for debugging/programming
3. **Nucleo-F429ZI board** or compatible STM32F429ZIT6 target

### Build Process
1. **Import Project**: File → Import → Existing Projects into Workspace
2. **Build**: Right-click project → Build Project (or Ctrl+B)
3. **Flash**: Right-click project → Run As → STM32 C/C++ Application
4. **Debug**: Use "nucleo Debug.launch" configuration

### Build Outputs
- **nucleo.elf**: Main executable binary
- **nucleo.map**: Memory layout and symbol information
- **nucleo.list**: Assembly listing with addresses

## Development Workflow

### Current Capabilities
- ✅ Basic GPIO control (LED blinking)
- ✅ HAL library integration
- ✅ Debug configuration ready
- ✅ Printf debug output
- ✅ STM32CubeMX integration for easy peripheral configuration

### Learning Path Integration
The project includes a comprehensive **STM32F429ZIT6_Firmware_Roadmap.md** that provides:
- **Phase 1-6 Learning Structure**: From beginner to guru level (12-18 months)
- **Detailed Peripheral Coverage**: GPIO, Timers, UART, SPI, I2C, DMA, USB, Ethernet
- **Advanced Topics**: RTOS, power management, DSP, motor control
- **Professional Skills**: Debugging, optimization, testing, bootloader development

### Next Development Steps
1. **Clock Optimization**: Increase system clock to 180MHz using PLL
2. **UART Communication**: Add serial communication for better debugging
3. **Timer-based PWM**: Replace simple delays with timer-controlled LED patterns
4. **Button Input**: Add user button interrupt handling
5. **Advanced Peripherals**: Implement UART, SPI, I2C based on roadmap phases

### Debug and Development Tools
- **SWD Debug Interface**: Hardware debugging with breakpoints
- **SWV Trace**: Printf output via Single Wire Viewer
- **Live Expressions**: Real-time variable monitoring
- **Exception Handling**: Configured for division by zero and hard fault detection

## Memory Configuration

### Flash Memory Layout (2MB total)
- **Application Code**: Starting at 0x08000000
- **Vector Table**: Beginning of flash
- **Available Space**: ~2MB for application and data

### SRAM Layout (256KB total)
- **Main SRAM**: 192KB at 0x20000000
- **CCM RAM**: 64KB at 0x10000000 (Core Coupled Memory for critical data)
- **Stack Size**: 1KB (configurable)
- **Heap Size**: 512 bytes (configurable)

## Documentation References

### Official Documentation
- **Reference Manual**: RM0090 (STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439)
- **Datasheet**: STM32F429ZI specific electrical characteristics
- **Programming Manual**: PM0214 (STM32 Cortex-M4 with FPU programming manual)

### Development Resources
- STM32CubeMX for peripheral configuration
- STM32CubeIDE for development and debugging
- ST Community forums for support
- Included comprehensive learning roadmap for systematic skill development

This project serves as an excellent foundation for learning STM32 development, with clear upgrade paths defined in the included roadmap for advancing from basic GPIO control to professional embedded systems development.