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
# define hal_init_core1() ((void)0)
# define hal_enable_irq(save)  ((void)0)
# define hal_disable_irq() (0)
# define hal_idle_cpu()    (sleep_ms(1), (get_procid() == 0) ? mrbc_tick_increment() : ((void) 0), mrbc_task_switch())

#endif

// get exclusive used spinlocks. If failed, get shared spinlocks.
# define vm_mutex_init(lock_num)      (lock_num > 0 ? spin_lock_init(lock_num) : spin_lock_init(next_striped_spin_lock_num()))
# define vm_mutex_lock(mutex)         (spin_lock_blocking(mutex))
# define vm_mutex_unlock(mutex, save) (spin_unlock(mutex, save))
# define get_procid()                 (get_core_num())

/***** Typedefs *************************************************************/
typedef uint32_t interrupt_status_t;


/***** Global variables *****************************************************/
extern volatile uint32_t doorbell_irq;

// At RP2350, max number of spinlocks (exclusive use) is 8 (24-31).
extern spin_lock_t * alloc_mutex;
extern spin_lock_t * write_mutex;
extern spin_lock_t * gc_mutex;
extern spin_lock_t * globalkv_mutex;
extern spin_lock_t * symbol_mutex;
extern spin_lock_t * coresending_mutex;

// At RP2350, max number of spinlocks (shared) is 8 (16-23).

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

/***** Local headers ********************************************************/
#include "rrt0.h"


#ifdef __cplusplus
}
#endif
#endif // ifndef MRBC_HAL_H_
