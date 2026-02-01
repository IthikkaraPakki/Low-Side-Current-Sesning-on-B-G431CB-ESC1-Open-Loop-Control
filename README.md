# Low Side Current Sensing using STM32G4 - Open Loop Speed Control

This repo discusses the capability of STM32 to read analog values such as voltage and current in Control applications. Here an open loop speed control is implemented for a BLDC with Low Side Current sensing.

You need a decent understanding of 3 phase 3 half-bridge inverter, BLDC and Sine Modulation to understand the concepts. But who is stopping you. Give it a try!

For a poly phase or 3 phase motor, current measurement can be done using different topologies
* Inline Current 
* High Side Current Sensing
* Low Side Current sensing - Measurements are taken when low side switches are on

The project specifically uses a B-G431CB-ESC1 evaluation board, which has a stm32G431cbu6, onboard gate drivers and MOSFETs. It also has a current sensing shunt resistors of 0.003 ohms. The board supports up to 40A of rated current with forced cooling. But in the setup here, the motor rarely draws 1A.

ADC are of 12-bit, so your readings will be between 0-(2^12 or 4096) corresponding to 0-3.3v. So yes, for current sensing we measure voltage across the current sensing element(here resistors) and calculate the value using ohms law (V = I*R). Also the sensing circuit is later biased with 2.2K/22K ohm voltage divider circuit. So at 0 current your adc will read 10% of 4096. This is to enable -ve current readings.

Only 2 phase current are read in the project. This is because the third current vector can usually be calculated for a balanced circuit like BLDC. Also, no current conversion from 12-bit to actual value or any amplification of sensed signal using any amplifier(Op-amp) is done here. User can implement it according to their hardware. STM32G4 series usually comes with high performance internal Op-amp.

* Here R = 0.003(Sensing element value)

* G = gain of amplifier

* Vref = reference voltage into adc(here 3.3v)

* $V = \frac{Raw ADC Value}{4096 * G} * Vref$,  Ignore G or keep it 1 if no amplifier is used

* $I = \frac{V}{R}$ 

Also this project using timers and ADC ISRs to implement Open Loop Control Logic, ADC read and USART transmission. This is ensure determinism or strict timing targets which is usually how control logic are implemented in embedded hardware.

This project also supports USART transmission for data logging. Data frame format is described at the end and follows this MATHWORKS example: https://in.mathworks.com/help/ecoder/stmicroelectronicsstm32f4discovery/ug/fast-serial-data-monitoring-two-model-approach-example.html

I have used "stm32_fast_serial_datalogging_host" simulink model to monitor the current readings from USART in realtime, but you can simple generate a python/matlab script using any AI tools to visualize.

### Files to look for in the repo, rest CubeIDE generated
* main.c
* openloop_isr.c
* openloop.h
* stm32g4xx_it.c
* LowSideCurrentSense_OpenLoopV3.ioc ( Hardware Config)

## Structure of algorithm

```
┌─────────────────────────────────────────────────────────────────────┐
│                         HARDWARE TRIGGERS                           │
│                                                                     │
│  TIM1 (20kHz)                ADC1                    Motor Control  │
│  ┌──────────┐               ┌──────┐                 ┌────────────┐ │
│  │ Counter  │──TRGO────────>│Inject│──JEOS IRQ─────> │ Algorithm  │ │
│  │ Update   │   (HW)        │Conv. │   (20kHz)       │ Execution  │ │
│  └──────────┘               └──────┘                 └────────────┘ │
│       │                        │                           │        │
│       │ PWM Out                │ Read Ia, Ib               │ Update │
│       ▼                        ▼                           ▼        │
│   [3-Phase]              [ADC Values]                [PWM Duty]     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```
```
TIM1 Counter (Center-Aligned Mode 1)
┌────────────────────────────────────────┐
│  Count: 0 → 4250 → 0 → 4250 → 0 ...    │
│         ↑         ↑         ↑          │
│       Start     ARR       ARR          │
│                (Peak)    (Peak)        │
│                  │         │           │
│                  ▼         ▼           │
│            Update Event (UEV)          │
│         (Every 50 μs = 20 kHz)         │
│                  │                     │
│                  ├─> TRGO Signal       │
│                  │                     │
│                  ▼                     │
│         ADC1 Injected Trigger          │
│         (Hardware - No CPU)            │
│                  │                     │
│                  ▼                     │
│         ADC Conversion Start           │
│         IN15 (Ia), IN2 (Ib)            │
│         (~0.56 μs conversion)          │
│                  │                     │
│                  ▼                     │
│         JEOS Flag Set                  │
│                  │                     │
│                  ▼                     │
│       ADC1_2_IRQHandler()              │
└────────────────────────────────────────┘
```

```

ADC1_2_IRQHandler()
  │
  ├─> Check JEOS flag
  │     └─> if (LL_ADC_IsActiveFlag_JEOS(ADC1))
  │
  ├─> READ ADC VALUES
  │     ├─> ia_adc = Read IN15 (12-bit)
  │     └─> ib_adc = Read IN2 (12-bit)
  │
  ├─> MOTOR CONTROL ALGORITHM
  │     │
  │     ├─> Phase Accumulator
  │     │     ├─> theta += delta_theta
  │     │     └─> if (theta >= 2π) theta -= 2π
  │     │
  │     ├─> 3-Phase Sine Generation
  │     │     ├─> sin_a = sin_lut[theta]
  │     │     ├─> sin_b = sin_lut[theta + 120°]
  │     │     └─> sin_c = sin_lut[theta + 240°]
  │     │
  │     ├─> Amplitude Scaling
  │     │     ├─> sin_a *= amplitude
  │     │     ├─> sin_b *= amplitude
  │     │     └─> sin_c *= amplitude
  │     │
  │     └─> Sine → PWM Conversion
  │           ├─> pwm_a = (sin_a + 1.0) × ARR/2
  │           ├─> pwm_b = (sin_b + 1.0) × ARR/2
  │           └─> pwm_c = (sin_c + 1.0) × ARR/2
  │
  ├─> BUFFER ADC DATA
  │     │
  │     ├─> Calculate buffer index
  │     │     └─> base = HEADER_BYTES + (sample_idx × 4)
  │     │
  │     ├─> Store samples (little-endian)
  │     │     ├─> buf[base+0] = ia_adc & 0xFF
  │     │     ├─> buf[base+1] = ia_adc >> 8
  │     │     ├─> buf[base+2] = ib_adc & 0xFF
  │     │     └─> buf[base+3] = ib_adc >> 8
  │     │
  │     └─> sample_idx++
  │
  ├─> CHECK IF BUFFER FULL
  │     │
  │     └─> if (sample_idx >= 600)
  │           │
  │           ├─> Reset sample_idx = 0
  │           │
  │           ├─> Add Packet Framing
  │           │     ├─> Header: buf[0:1] = 0x5353 ('SS')
  │           │     └─> Footer: buf[2402:2403] = 0x6565 ('ee')
  │           │
  │           └─> Trigger UART Transmission
  │                 ├─> uart_tx_busy = 1
  │                 ├─> uart_tx_idx = 0
  │                 └─> Enable USART2 TXE interrupt
  │
  ├─> UPDATE PWM OUTPUTS
  │     ├─> TIM1->CCR1 = pwm_a
  │     ├─> TIM1->CCR2 = pwm_b
  │     └─> TIM1->CCR3 = pwm_c
  │
  │
  └─> Clear JEOS flag

  ```

## Data Structures
### **UART Packet Format**

```
Byte Index:  [0] [1] [2] [3] [4] [5] ... [2400][2401] [2402][2403]
            ┌───┬───┬───┬───┬───┬───┬───┬─────┬─────┬─────┬─────┐
Content:    │'S'│'S'│Ia0│Ia0│Ib0│Ib0│...│Ia599│Ib599│ 'e' │ 'e' │
            │   │   │ L │ H │ L │ H │   │ L H │ L H │     │     │
            └───┴───┴───┴───┴───┴───┴───┴─────┴─────┴─────┴─────┘
             Header    Sample 0          Sample 599    Footer
            (0x5353)   (4 bytes)         (4 bytes)    (0x6565)

Total: 2404 bytes

```

### **Sample Data Format (Little-Endian)**

```
Each Sample (4 bytes):
┌────────┬────────┬────────┬────────┐
│ Ia_Low │ Ia_High│ Ib_Low │ Ib_High│
│ [7:0]  │ [11:8] │ [7:0]  │ [11:8] │
└────────┴────────┴────────┴────────┘
  byte 0   byte 1   byte 2   byte 3

Example:
  ia_adc = 0x0A5F (2655 decimal)
  ib_adc = 0x0B3C (2876 decimal)
  
  Buffer: [5F][0A][3C][0B]

```

---

## Performance Metrics

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Motor Control Frequency** | 20 kHz | PWM update rate |
| **ADC Sampling Frequency** | 20 kHz | Synchronized to PWM peak |
| **PWM Frequency** | 10 kHz | Full triangle period |
| **Data Logging Rate** | 33.3 packets/sec | 600 samples each of uint16_t @ 20 kHz |
| **Streaming** | 80KB/sec | Total 1200 samples of uint16_t |
| **UART Bandwidth** | 2 Mbps | Sufficient for real-time |
| **Packet Overhead** | 0.17% | 4 bytes / 2404 bytes |
| **Dead Time** | 700 ns | Prevents shoot-through |

---


## Support

Connect with me at https://www.linkedin.com/in/bilbert-george-85a505261/
