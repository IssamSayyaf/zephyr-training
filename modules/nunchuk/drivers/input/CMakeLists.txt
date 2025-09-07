# SPDX-License-Identifier: Apache-2.0

# Only compile the driver if enabled
zephyr_sources_ifdef(CONFIG_NUNCHUK_DRIVER nunchuk.c)