# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LTR390)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LTR390 src/ltr390.c)

endif(CONFIG_LTR390)