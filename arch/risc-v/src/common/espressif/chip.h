/****************************************************************************
 * arch/risc-v/src/common/espressif/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_CHIP_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp_memorymap.h"

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Section for exception handler. */

#define EXCEPTION_SECTION .iram1

#ifdef __ASSEMBLY__

#if defined(CONFIG_ARCH_CHIP_ESP32P4) && defined(CONFIG_SMP) && \
    CONFIG_ARCH_INTERRUPTSTACK > 15
.macro setintstack tmp0, tmp1
  up_cpu_index \tmp0
  li    \tmp1, STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)
  mul   \tmp1, \tmp0, \tmp1
  la    \tmp0, g_intstacktop
  sub   \tmp0, \tmp0, \tmp1
  li    \tmp1, STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)
  sub   \tmp1, \tmp0, \tmp1
  blt   sp, \tmp1, 1f
  bgt   sp, \tmp0, 1f
  j     2f
1:
  mv    sp, \tmp0
2:
.endm
#endif

#else

#if defined(CONFIG_ARCH_CHIP_ESP32P4) && defined(CONFIG_SMP)
void esp_ipi_send(int cpu);
void esp_ipi_clear(int cpu);
void esp_ipi_wait(int cpu);
void esp_ipi_initialize(int cpu);
void esp_smp_start_secondary(int cpu);
void esp_smp_secondary_start(void) noreturn_function;
#endif

#endif

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_CHIP_H */
