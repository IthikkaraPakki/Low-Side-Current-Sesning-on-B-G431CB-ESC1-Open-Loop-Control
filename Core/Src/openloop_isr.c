#include "openloop.h"
#include "main.h"
#include "stm32g4xx_ll_usart.h"
#include <math.h>

/* ===================== GLOBAL DEFINITIONS ===================== */

volatile float target_rpm = 0.0f;
volatile float amplitude  = 0.0f;

volatile float elec_freq_hz = 0.0f;
volatile float delta_theta  = 0.0f;

volatile float theta = 0.0f;

volatile uint16_t pwm_a = 0;
volatile uint16_t pwm_b = 0;
volatile uint16_t pwm_c = 0;

/* ===================== ADC & UART VARIABLES ===================== */

volatile uint16_t ia_adc = 0;
volatile uint16_t ib_adc = 0;

volatile uint8_t  uart_tx_buf[TOTAL_PACKET_BYTES];
volatile uint16_t sample_idx = 0;
volatile uint8_t  packet_ready = 0;

volatile uint16_t uart_tx_idx = 0;
volatile uint8_t  uart_tx_busy = 0;

/* ===================== SINE LUT ===================== */

static float sin_lut[LUT_SIZE];

void init_sine_lut(void)
{
    for (uint16_t i = 0; i < LUT_SIZE; i++)
    {
        sin_lut[i] = sinf(TWO_PI * (float)i / (float)LUT_SIZE);
    }
}

static inline float sin_from_lut(float angle)
{
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f)    angle += TWO_PI;

    uint16_t index = (uint16_t)((angle * LUT_SIZE) / TWO_PI);
    return sin_lut[index];
}

/* ===================== ISR ===================== */

void motor_control_isr(void)
{
    /* Phase accumulator */
    theta += delta_theta;
    if (theta >= TWO_PI)
        theta -= TWO_PI;

    /* 3-phase sine */
    float sin_a = sin_from_lut(theta);
    float sin_b = sin_from_lut(theta + (2.0f * TWO_PI / 3.0f));
    float sin_c = sin_from_lut(theta + (4.0f * TWO_PI / 3.0f));

    /* Amplitude scaling */
    sin_a *= amplitude;
    sin_b *= amplitude;
    sin_c *= amplitude;

    /* Sine → PWM */
    pwm_a = (uint16_t)((sin_a + 1.0f) * (PWM_ARR * 0.5f));
    pwm_b = (uint16_t)((sin_b + 1.0f) * (PWM_ARR * 0.5f));
    pwm_c = (uint16_t)((sin_c + 1.0f) * (PWM_ARR * 0.5f));

    /* Buffer ADC samples for UART transmission (offset by header) */
    uint16_t base = HEADER_BYTES + (sample_idx * BYTES_PER_SAMPLE);
    uart_tx_buf[base + 0] = ia_adc & 0xFF;
    uart_tx_buf[base + 1] = ia_adc >> 8;
    uart_tx_buf[base + 2] = ib_adc & 0xFF;
    uart_tx_buf[base + 3] = ib_adc >> 8;

    sample_idx++;

    /* When buffer is full, add header/footer and trigger UART transmission */
    if (sample_idx >= PACKET_SAMPLES)
      {
         sample_idx = 0;
         packet_ready = 1;

         /* Add header */
         uart_tx_buf[0] = PACKET_HEADER & 0xFF;
         uart_tx_buf[1] = PACKET_HEADER >> 8;

         /* Add footer */
         uart_tx_buf[HEADER_BYTES + PACKET_BYTES + 0] = PACKET_FOOTER & 0xFF;
         uart_tx_buf[HEADER_BYTES + PACKET_BYTES + 1] = PACKET_FOOTER >> 8;

         /* Start UART transmission if not already busy */
         if (!uart_tx_busy)
           {
             uart_tx_busy = 1;
             uart_tx_idx = 0;
             packet_ready = 0;
             LL_USART_EnableIT_TXE(USART2);
           }
       }
}

void update_open_loop_params(float rpm_cmd, float amp_cmd)
{
    target_rpm = rpm_cmd;
    amplitude  = amp_cmd;

    elec_freq_hz = (target_rpm * POLE_PAIRS) / 60.0f;
    delta_theta  = (TWO_PI * elec_freq_hz) / ISR_FREQ_HZ;
}
