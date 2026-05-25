/*! @file
  @brief
  Hardware abstraction layer
        for RP2350

  <pre>
  Copyright (C) 2015- Kyushu Institute of Technology.
  Copyright (C) 2015- Shimane IT Open-Innovation Center.

  This file is distributed under BSD 3-Clause License.
  </pre>
*/

/***** Feature test switches ************************************************/
/***** System headers *******************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/***** Local headers ********************************************************/
#include "hal.h"

/***** Constat values *******************************************************/
/***** Macros ***************************************************************/
/***** Typedefs *************************************************************/
/***** Function prototypes **************************************************/
/***** Local variables ******************************************************/
struct repeating_timer timer0;

volatile static int doorbell_counter;

volatile static uint8_t is_core0_busy = true;

/***** Global variables *****************************************************/
spin_lock_t * alloc_mutex;
spin_lock_t * write_mutex;
spin_lock_t * gc_mutex;
spin_lock_t * globalkv_mutex;
spin_lock_t * symbol_mutex;
spin_lock_t * coresending_mutex; 
spin_lock_t * task_mutex;


volatile uint32_t doorbell_irq;

/***** Signal catching functions ********************************************/
/***** Local functions ******************************************************/
#ifdef MRBC_MULTICORE

void multicore_init_core0(void)
{
  alloc_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  write_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  gc_mutex = vm_mutex_init(spin_lock_claim_unused(false)); 
  globalkv_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  symbol_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  coresending_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  task_mutex = vm_mutex_init(spin_lock_claim_unused(false));

  multicore_lockout_victim_init();
}

#else // #ifdef MRBC_MULTICORE

void multicore_init_core0(void)
{
  
}


#endif // #ifdef MRBC_MULTICORE

/***** Global functions *****************************************************/
#if !defined(MRBC_NO_TIMER) && defined(MRBC_MULTICORE)
//================================================================
/*!@brief
  timer alarm irq (for both busy and sleep, at core1)

*/
void alarm_irq_core1(void)
{
  hal_disable_irq();
  multicore_doorbell_clear_current_core(doorbell_counter);
  mrbc_task_switch();
  hal_enable_irq();
}

//================================================================
/*!@brief
  initialize for core1

*/
void hal_init_core1(void)
{
  doorbell_irq = multicore_doorbell_irq_num(doorbell_counter);
  irq_set_exclusive_handler(doorbell_irq, alarm_irq_core1);
  irq_set_enabled(doorbell_irq, true);
  multicore_lockout_victim_init();
}

#elifdef MRBC_MULTICORE //#if !defined(MRBC_NO_TIMER) && defined(MRBC_MULTICORE)
void hal_init_core1(void)
{
  multicore_lockout_victim_init();
}

#endif //#elifdef MRBC_MULTICORE 

#ifndef MRBC_NO_TIMER

//================================================================
/*!@brief
  timer alarm irq (for busy, at core0)

*/
bool alarm_irq(struct repeating_timer *t)
{
  if ( is_core0_busy ) {
    mrbc_tick_increment();

    notify_other_core(doorbell_counter);

    mrbc_task_switch();
  }

  return true;
}

//================================================================
/*!@brief
  timer alarm irq (for sleep, at core0)

*/
static inline void alarm_irq_at_sleep(void)
{
  if ( !is_core0_busy ) {
    mrbc_tick_increment();

    notify_other_core(doorbell_counter);

    mrbc_task_switch();
  }
  is_core0_busy = true; 
}

//================================================================
/*!@brief
  initialize

*/
void hal_init(void)
{
  powman_timer_set_1khz_tick_source_lposc();

  add_repeating_timer_ms(1, alarm_irq, NULL, &timer0);
  powman_timer_start();

  clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS
                        | CLOCKS_SLEEP_EN0_CLK_SYS_SIO_BITS;
  clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS
                        | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_USB_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS;

  doorbell_counter = notifier_init();
  if (doorbell_counter == -1) {
    char msg[] = "doorbell claim is failed!";
    hal_write(1, msg, sizeof(msg));
    exit(1);
  }

  multicore_init_core0();
}

//================================================================
/*!@brief
  Flush write buffer

  @param  fd    dummy, but 1.
*/
int hal_flush(int fd)
{
  return 0;
}

#else // #ifndef MRBC_NO_TIMER
void hal_init(void)
{
  multicore_init_core0();
}

#endif // #ifndef MRBC_NO_TIMER

#if !defined(MRBC_NO_TIMER) && defined(MRBC_MULTICORE)
//================================================================
/*!@brief
  Go to sleep mode with alarm after 1ms.
*/
void goto_sleep_for_1ms()
{
  struct timespec ts;
  if (get_procid() == 0) {
    is_core0_busy = false;
    aon_timer_get_time(&ts);

    ts.tv_nsec += 1e6;
    if (ts.tv_nsec >= 1e9)
    {
      ts.tv_sec += 1;
      ts.tv_nsec -= 1e9;
    }

    aon_timer_enable_alarm(&ts, alarm_irq_at_sleep, true);
    
    __wfi();
    // To ensure that the AON timer wakes up the sleep state
    while(!is_core0_busy) {
      tight_loop_contents();
    }
  } else {
    __wfi();
  }
}
#elifndef MRBC_NO_TIMER
//================================================================
/*!@brief
  Go to sleep mode with alarm after 1ms.
*/
void goto_sleep_for_1ms()
{
  struct timespec ts;
  is_core0_busy = false;
  aon_timer_get_time(&ts);

  ts.tv_nsec += 1e6;
  if (ts.tv_nsec >= 1e9)
  {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1e9;
  }

  aon_timer_enable_alarm(&ts, alarm_irq_at_sleep, true);
  
  __wfi();
  // To ensure that the AON timer wakes up the sleep state
  while(!is_core0_busy) {
    tight_loop_contents();
  }
}
#endif

//================================================================
/*!@brief
  Write

  @param  fd    dummy, but 1.
  @param  buf   pointer of buffer.
  @param  nbytes        output byte length.

  Memo: Steps to use uart_putc_raw() with hal_write.
  1. Write in main function↓
    uart_init(uart0,115200);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);

  2. Comment out the putchar for hal_write.
  3. Uncomment uart_putc_raw for hal_write.
*/
int hal_write(int fd, const void *buf, int nbytes)
{
  int i = nbytes;
  const uint8_t *p = buf;
  
  interrupt_status_t save;
  save = vm_mutex_lock( write_mutex );

  while (--i >= 0)
  {
    putchar(*p++);
    // uart_putc_raw(uart0, *p++ );
  }
  
  vm_mutex_unlock( write_mutex, save );
  
  return nbytes;
}

//================================================================
/*!@brief
  abort program

  @param s	additional message.
*/
void hal_abort(const char *s)
{
  if (s)
  {
    hal_write(1, s, strlen(s));
  }

  abort();
}
