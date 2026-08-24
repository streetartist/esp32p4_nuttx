/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_smp.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_SMP

#include <assert.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "chip.h"
#include "riscv_internal.h"
#include "esp_irq.h"

#include "esp_err.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_ipc.h"
#include "hal/cpu_utility_ll.h"
#include "hal/crosscore_int_ll.h"
#include "soc/system_intr.h"

extern void ets_set_appcpu_boot_addr(uint32_t start);
extern int _vector_table;
extern int _mtvt_table;

static bool g_ipi_initialized[CONFIG_SMP_NCPUS];

struct esp_ipc_call_s
{
  esp_ipc_func_t func;
  void *arg;
};

static int esp_ipc_call_adapter(void *arg)
{
  struct esp_ipc_call_s *call = arg;

  call->func(call->arg);
  return OK;
}

static esp_err_t esp_ipc_call_internal(uint32_t cpu_id,
                                       esp_ipc_func_t func, void *arg)
{
  struct esp_ipc_call_s call;
  int ret;

  if (cpu_id >= CONFIG_SMP_NCPUS || func == NULL)
    {
      return ESP_ERR_INVALID_ARG;
    }

  call.func = func;
  call.arg = arg;
  ret = nxsched_smp_call_single(cpu_id, esp_ipc_call_adapter, &call);

  return ret < 0 ? ESP_FAIL : ESP_OK;
}

esp_err_t esp_ipc_call(uint32_t cpu_id, esp_ipc_func_t func, void *arg)
{
  return esp_ipc_call_internal(cpu_id, func, arg);
}

esp_err_t esp_ipc_call_blocking(uint32_t cpu_id, esp_ipc_func_t func,
                                void *arg)
{
  return esp_ipc_call_internal(cpu_id, func, arg);
}

void esp_ipi_send(int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
  crosscore_int_ll_trigger_interrupt(cpu);
}

void esp_ipi_clear(int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
  crosscore_int_ll_clear_interrupt(cpu);
}

void esp_ipi_wait(int cpu)
{
  DEBUGASSERT(cpu > 0 && cpu < CONFIG_SMP_NCPUS);

  while (crosscore_int_ll_get_state(cpu) == 0)
    {
      __asm__ __volatile__("nop");
    }

  esp_ipi_clear(cpu);
}

void esp_ipi_initialize(int cpu)
{
  int irq;
  int source;
  int cpuint;

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if (g_ipi_initialized[cpu])
    {
      return;
    }

  if (cpu == 0)
    {
      esp_ipi_clear(cpu);
    }

  source = cpu == 0 ? SYS_CPU_INTR_FROM_CPU_0_SOURCE :
                      SYS_CPU_INTR_FROM_CPU_1_SOURCE;
  irq = cpu == 0 ? ESP_IRQ_FROM_CPU_INTR0 : ESP_IRQ_FROM_CPU_INTR1;

  cpuint = esp_setup_irq(source, ESP_IRQ_PRIORITY_1,
                         ESP_IRQ_TRIGGER_LEVEL,
                         riscv_smp_call_handler, NULL);
  DEBUGASSERT(cpuint >= 0);

  g_ipi_initialized[cpu] = true;
  up_enable_irq(irq);
}

void esp_smp_start_secondary(int cpu)
{
  DEBUGASSERT(cpu == 1);

  esp_cpu_unstall(cpu);
  cpu_utility_ll_enable_clock_and_reset_app_cpu();
  cpu_utility_ll_enable_clock_and_reset_app_cpu_int_matrix();
  ets_set_appcpu_boot_addr((uint32_t)esp_smp_secondary_start);
}

void IRAM_ATTR esp_smp_secondary_c(void)
{
  ets_set_appcpu_boot_addr(0);
  esp_cpu_intr_set_ivt_addr(&_vector_table);
  esp_cpu_intr_set_mtvt_addr(&_mtvt_table);
  riscv_cpu_boot(1);
  __builtin_unreachable();
}

void IRAM_ATTR __attribute__((naked, noinline)) esp_smp_secondary_start(void)
{
  __asm__ __volatile__(
    ".option push\n"
    ".option norelax\n"
    "la gp, __global_pointer$\n"
    "la t0, g_idle_topstack\n"
    "lw sp, 0(t0)\n"
    "andi sp, sp, -16\n"
    ".option pop\n"
    "tail esp_smp_secondary_c\n");
}

#endif /* CONFIG_SMP */
