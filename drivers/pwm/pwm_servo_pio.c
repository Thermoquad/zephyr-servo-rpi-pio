/*
 * Copyright (c) 2026 Thermoquad
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <hardware/pio.h>
#include <hardware/clocks.h>

#define DT_DRV_COMPAT raspberrypi_pico_pwm_servo_pio

struct servo_pio_config {
	const struct device *piodev;
	uint32_t pin;
};

struct servo_pio_data {
	size_t sm;
	uint32_t offset;
};

/*
 * Continuous PWM PIO program.
 *
 * Packed 32-bit pulse descriptor pulled from TX FIFO each cycle:
 *   bits[15:0]  = high-time delay   = pulse_us - 3
 *   bits[31:16] = low-time delay    = low_us   - 6
 *
 * `pull noblock` refreshes OSR from the FIFO; if FIFO is empty it copies
 * scratch X back into OSR. `mov x, osr` re-caches the descriptor after
 * the pull, so a single pushed value re-circulates forever and pushing
 * a new value transparently updates the pulse on the next cycle.
 *
 * `set pindirs, 1` at the top of every cycle keeps the pin in output
 * mode independent of how the host-side init helpers interact with
 * SMx_INSTR.
 *
 * Out shift right (LSB first), autopull disabled, clock = 1 µs/tick.
 *
 * NOTE: A `jmp y--, N` placed at WRAP would conflict with the wrap rule
 * (wrap overrides jump destination), so the pull/cache pair sits at the
 * end of the program and the delay loops live in the interior.
 *
 * Assembly (wrap_target=0, wrap=8):
 *   0: set  pindirs, 1   ; pin direction = output
 *   1: out  y, 16        ; Y := high-time delay
 *   2: set  pins, 1      ; drive pin high
 *   3: jmp  y--, 3       ; high-delay loop
 *   4: out  y, 16        ; Y := low-time delay
 *   5: set  pins, 0      ; drive pin low
 *   6: jmp  y--, 6       ; low-delay loop
 *   7: pull noblock      ; OSR := FIFO entry, or X if FIFO empty
 *   8: mov  x, osr       ; cache descriptor; wraps to 0
 */
RPI_PICO_PIO_DEFINE_PROGRAM(servo_pwm, 0, 8,
	0xe081, /*  0: set    pindirs, 1   */
	0x6050, /*  1: out    y, 16        */
	0xe001, /*  2: set    pins, 1      */
	0x0083, /*  3: jmp    y--, 3       */
	0x6050, /*  4: out    y, 16        */
	0xe000, /*  5: set    pins, 0      */
	0x0086, /*  6: jmp    y--, 6       */
	0x8080, /*  7: pull   noblock      */
	0xa027  /*  8: mov    x, osr       */
);

static int servo_pio_set_cycles(const struct device *dev, uint32_t channel,
				uint32_t period, uint32_t pulse, pwm_flags_t flags)
{
	const struct servo_pio_config *config = dev->config;
	struct servo_pio_data *data = dev->data;
	PIO pio = pio_rpi_pico_get_pio(config->piodev);

	if (channel != 0) {
		return -EINVAL;
	}

	if (flags & PWM_POLARITY_INVERTED) {
		pulse = period - pulse;
	}

	if (pulse > period) {
		return -EINVAL;
	}

	uint32_t low = period - pulse;

	/*
	 * Pin-high cycles = N + 3 (PC=2 set + PC=3 loop iters + PC=4 out).
	 * Pin-low  cycles = M + 6 (PC=5 set + PC=6 loop iters + PC=7 pull +
	 *                          PC=8 mov + PC=0 set pindirs + PC=1 out).
	 * Solving:  N = pulse - 3,  M = low - 6.
	 */
	uint32_t n = (pulse > 3) ? (pulse - 3) : 0;
	uint32_t m = (low > 6)   ? (low - 6)   : 0;
	uint32_t word = ((m & 0xFFFF) << 16) | (n & 0xFFFF);

	pio_sm_put(pio, data->sm, word);

	return 0;
}

static int servo_pio_get_cycles_per_sec(const struct device *dev,
					uint32_t channel, uint64_t *cycles)
{
	if (channel != 0) {
		return -EINVAL;
	}

	/* Clock divider configured for 1MHz → 1µs per tick */
	*cycles = 1000000;
	return 0;
}

static int servo_pio_init(const struct device *dev)
{
	const struct servo_pio_config *config = dev->config;
	struct servo_pio_data *data = dev->data;
	PIO pio;
	size_t sm;
	int ret;

	pio = pio_rpi_pico_get_pio(config->piodev);

	ret = pio_rpi_pico_allocate_sm(config->piodev, &sm);
	if (ret < 0) {
		return ret;
	}
	data->sm = sm;

	if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(servo_pwm))) {
		return -EBUSY;
	}

	uint32_t offset = pio_add_program(pio,
					  RPI_PICO_PIO_GET_PROGRAM(servo_pwm));
	data->offset = offset;

	pio_sm_config sm_config = pio_get_default_sm_config();

	sm_config_set_set_pins(&sm_config, config->pin, 1);
	sm_config_set_out_shift(&sm_config, true, false, 32);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(servo_pwm),
			   offset + RPI_PICO_PIO_GET_WRAP(servo_pwm));

	/* 1µs per tick: divider = sys_clk / 1MHz */
	float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
	sm_config_set_clkdiv(&sm_config, div);

	pio_gpio_init(pio, config->pin);
	pio_sm_set_consecutive_pindirs(pio, sm, config->pin, 1, true);
	pio_sm_set_pins_with_mask(pio, sm, 0, BIT(config->pin));
	pio_sm_init(pio, sm, offset, &sm_config);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static DEVICE_API(pwm, servo_pio_driver_api) = {
	.set_cycles = servo_pio_set_cycles,
	.get_cycles_per_sec = servo_pio_get_cycles_per_sec,
};

#define SERVO_PIO_INIT(idx)							\
	static const struct servo_pio_config servo_pio##idx##_config = {		\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(idx)),			\
		.pin = DT_INST_GPIO_PIN(idx, gpios),				\
	};									\
	static struct servo_pio_data servo_pio##idx##_data;			\
										\
	DEVICE_DT_INST_DEFINE(idx, servo_pio_init, NULL,			\
			      &servo_pio##idx##_data,				\
			      &servo_pio##idx##_config, POST_KERNEL,		\
			      CONFIG_PWM_INIT_PRIORITY,				\
			      &servo_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SERVO_PIO_INIT)
