/*! @file
  @brief
  Hardware abstraction layer
        for RP2040

  <pre>
  Copyright (C) 2015- Kyushu Institute of Technology.
  Copyright (C) 2015- Shimane IT Open-Innovation Center.

  This file is distributed under BSD 3-Clause License.
  </pre>
*/

#ifndef MRBC_SRC_HAL_H_
#define MRBC_SRC_HAL_H_

/***** Feature test switches ************************************************/
/***** System headers *******************************************************/
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/aon_timer.h"
#include "pico/sync.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/powman.h"
#include <stdatomic.h>
#include <assert.h>

/***** Constant values ******************************************************/
#define ALARM_IRQ TIMER0_IRQ_0

/***** Macros ***************************************************************/
#if !defined(MRBC_TICK_UNIT)
#define MRBC_TICK_UNIT_1_MS   1
#define MRBC_TICK_UNIT_2_MS   2
#define MRBC_TICK_UNIT_4_MS   4
#define MRBC_TICK_UNIT_10_MS 10
// You may be able to reduce power consumption if you configure
// MRBC_TICK_UNIT_2_MS or larger.

#define ALLOC_MUTEX    0
#define WRITE_MUTEX    1
#define GC_MUTEX       2
#define GLOBALKV_MUTEX 3
#define SYMBOL_MUTEX   4
#define MUTEX_REQUIRE_NUM 5

#define MRBC_TICK_UNIT MRBC_TICK_UNIT_1_MS
// Substantial timeslice value (millisecond) will be
// MRBC_TICK_UNIT * MRBC_TIMESLICE_TICK_COUNT (+ Jitter).
// MRBC_TIMESLICE_TICK_COUNT must be natural number
// (recommended value is from 1 to 10).
#define MRBC_TIMESLICE_TICK_COUNT 10
#endif

#ifndef MRBC_NO_TIMER
void hal_init(void);
void hal_init_core1(void);
# define hal_enable_irq(save)  (restore_interrupts(save))
# define hal_disable_irq() (save_and_disable_interrupts())
# define hal_idle_cpu()    goto_sleep_for_1ms()
#else // MRBC_NO_TIMER
void hal_init(void);
#define hal_init_core1() ((void)0)
# define hal_enable_irq(save)  ((void)0)
# define hal_disable_irq() (0)
# define hal_idle_cpu()    (sleep_ms(1), (get_procid() == 0) ? mrbc_tick_increment() : ((void) 0), mrbc_task_switch())

#endif

# define get_procid() (get_core_num())

/***** Typedefs *************************************************************/
typedef uint32_t interrupt_status_t;
typedef struct {
  spin_lock_t * vm_mutex;
  volatile uint32_t is_available[MUTEX_REQUIRE_NUM];
} Monitor;


/***** Global variables *****************************************************/
extern Monitor mrbc_monitor;
extern uint32_t doorbell_irq;

/***** Function prototypes **************************************************/
#ifdef __cplusplus
extern "C" {
#endif

int hal_write(int fd, const void *buf, int nbytes);
int hal_flush(int fd);
void hal_abort(const char *s);
void alarm_init();
void goto_sleep_for_1ms();

/***** Inline functions *****************************************************/
//================================================================
/*!@brief
  Mutex lock for processing the VM itself

*/
static inline void vm_mutex_lock(const int resource)
{
  interrupt_status_t save;
  while (true) {
    save = spin_lock_blocking(mrbc_monitor.vm_mutex);
    if (mrbc_monitor.is_available[resource]) {
      mrbc_monitor.is_available[resource] = 0;
      spin_unlock(mrbc_monitor.vm_mutex, save);
      break;
    }
    spin_unlock(mrbc_monitor.vm_mutex, save);
    tight_loop_contents();
  }
}

//================================================================
/*!@brief
  Mutex unlock for processing the VM itself

*/
static inline void vm_mutex_unlock(const int resource)
{
  interrupt_status_t save;
  
  save = spin_lock_blocking(mrbc_monitor.vm_mutex);
  mrbc_monitor.is_available[resource] = 1;
  spin_unlock(mrbc_monitor.vm_mutex, save);
}

/***** Local headers ********************************************************/
#include "rrt0.h"


#ifdef __cplusplus
}
#endif
#endif // ifndef MRBC_HAL_H_
