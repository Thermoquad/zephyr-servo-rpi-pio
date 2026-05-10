# Copyright (c) 2026 Thermoquad
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_PWM_SERVO_RPI_PICO_PIO pwm_servo_pio.c)
