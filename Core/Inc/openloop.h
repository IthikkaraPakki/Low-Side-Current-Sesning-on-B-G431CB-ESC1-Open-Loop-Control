#ifndef OPENLOOP_H
#define OPENLOOP_H

#include <stdint.h>

/* ===================== USER CONFIG ===================== */

#define ISR_FREQ_HZ     20000.0f
#define POLE_PAIRS      7
#define PWM_ARR         4250
#define LUT_SIZE        1024

#define TWO_PI          6.28318530718f

/* ===================== UART DATA LOGGING CONFIG ===================== */

#define PACKET_SAMPLES     600
#define BYTES_PER_SAMPLE   4
#define PACKET_BYTES       (PACKET_SAMPLES * BYTES_PER_SAMPLE)
#define PACKET_HEADER      0x5353  // 'SS' in little-endian uint16
#define PACKET_FOOTER      0x6565  // 'ee' in little-endian uint16
#define HEADER_BYTES       2
#define FOOTER_BYTES       2
#define TOTAL_PACKET_BYTES (HEADER_BYTES + PACKET_BYTES + FOOTER_BYTES)

/* ===================== GLOBAL STATE ===================== */

/* Control inputs */
extern volatile float target_rpm;
extern volatile float amplitude;

/* Derived values */
extern volatile float elec_freq_hz;
extern volatile float delta_theta;

/* ISR state */
extern volatile float theta;

/* Output PWM values */
extern volatile uint16_t pwm_a;
extern volatile uint16_t pwm_b;
extern volatile uint16_t pwm_c;

/* ADC current readings */
extern volatile uint16_t ia_adc;
extern volatile uint16_t ib_adc;

/* UART buffering */
extern volatile uint8_t  uart_tx_buf[TOTAL_PACKET_BYTES];
extern volatile uint16_t sample_idx;
extern volatile uint8_t  packet_ready;
extern volatile uint16_t uart_tx_idx;
extern volatile uint8_t  uart_tx_busy;

/* ===================== API ===================== */

void init_sine_lut(void);
void update_open_loop_params(float rpm_cmd, float amp_cmd);
void motor_control_isr(void);

#endif /* OPENLOOP_H */
