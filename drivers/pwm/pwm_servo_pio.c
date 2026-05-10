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
 * PIO program: servo PWM with 1µs resolution.
 *
 * The SM pulls a 32-bit word from the TX FIFO:
 *   bits [31:16] = low time in ticks (µs)
 *   bits [15:0]  = pulse width in ticks (µs)
 *
 * Out shift is right-first (LSB first), autopull disabled.
 * After pull, we shift out 16 bits to X (pulse), then 16 bits to Y (low).
 *
 * Assembly (wrap_target=0, wrap=6):
 *   0: pull block         ; wait for new pulse/period data
 *   1: out  x, 16         ; x = pulse width ticks
 *   2: out  y, 16         ; y = low time ticks
 *   3: set  pins, 1       ; drive pin high
 *   4: jmp  x--, 4        ; delay x+1 cycles (pulse width)
 *   5: set  pins, 0       ; drive pin low
 *   6: jmp  y--, 6        ; delay y+1 cycles (low time)
 *                          ; wraps back to 0
 */
RPI_PICO_PIO_DEFINE_PROGRAM(servo_pwm, 0, 6,
	0x80a0, /*  0: pull   block           */
	0x6030, /*  1: out    x, 16           */
	0x6050, /*  2: out    y, 16           */
	0xe001, /*  3: set    pins, 1         */
	0x0044, /*  4: jmp    x--, 4          */
	0xe000, /*  5: set    pins, 0         */
	0x0086  /*  6: jmp    y--, 6          */
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
	 * Overhead per cycle: pull(1) + out(1) + out(1) + set(1) + set(1) = 5
	 * jmp x-- loop executes x+1 times (including the final failing jmp).
	 * jmp y-- loop executes y+1 times.
	 * Total cycle = 5 + (x+1) + (y+1) = x + y + 7
	 * We want: pulse_actual = x+1+1(set_low) doesn't add — set is outside.
	 *
	 * Actually: pin high for set(1) + jmp_loop(x+1) = x+2 cycles.
	 *           pin low for set(1) + jmp_loop(y+1) + pull(1) + 2*out(1) = y+5.
	 * Total period = (x+2) + (y+5) = x + y + 7.
	 *
	 * Solving: x = pulse - 2, y = low - 5.
	 */
	uint32_t x = (pulse > 2) ? (pulse - 2) : 0;
	uint32_t y = (low > 5) ? (low - 5) : 0;

	/* Pack: bits[15:0]=x (pulse), bits[31:16]=y (low) */
	uint32_t word = (y << 16) | (x & 0xFFFF);

	/* Drain FIFO and write new value — ensures immediate update */
	pio_sm_drain_tx_fifo(pio, data->sm);
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

	/* Load initial 50Hz / 1500µs center position */
	uint32_t pulse = 1500 - 2;
	uint32_t low = (20000 - 1500) - 5;
	pio_sm_put_blocking(pio, sm, (low << 16) | pulse);

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
