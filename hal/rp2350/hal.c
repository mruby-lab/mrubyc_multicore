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
struct repeating_timer timer1;

static int doorbell_counter;

static uint8_t is_core0_busy = true;

/***** Global variables *****************************************************/
uint32_t monitor_pre_canary[1];
Monitor mrbc_monitor;
uint32_t mrbc_monitor_post_canary[1];
uint32_t doorbell_irq;

/***** Signal catching functions ********************************************/
/***** Local functions ******************************************************/

/***** Global functions *****************************************************/
#ifndef MRBC_NO_TIMER

//================================================================
/*!@brief
  timer alarm irq (for busy, at core0)

*/
bool alarm_irq(struct repeating_timer *t)
{
  if (is_core0_busy) {
    mrbc_tick_increment();
    multicore_doorbell_set_other_core(doorbell_counter);
    mrbc_task_switch();
  }

  return true;
}

//================================================================
/*!@brief
  timer alarm irq (for sleep, at core0)

*/
void alarm_irq_at_sleep(void)
{
  if (is_core0_busy == false) {
    mrbc_tick_increment();
    multicore_doorbell_set_other_core(doorbell_counter);
    mrbc_task_switch();
    is_core0_busy = true;
  }
}

//================================================================
/*!@brief
  timer alarm irq (for both busy and sleep, at core1)

*/
void alarm_irq_core1(void)
{
  multicore_doorbell_clear_current_core(doorbell_counter);
  mrbc_task_switch();
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

  clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS;
  clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS
                        | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_USB_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS 
                        | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS;

  // If this can't get a spinlock, this causes panic.
  mrbc_monitor.vm_mutex = spin_lock_init(spin_lock_claim_unused(true));
  for (int i = 0; i < MUTEX_REQUIRE_NUM; i++) {
    mrbc_monitor.is_available[i] = true;
  }
  
  doorbell_counter = multicore_doorbell_claim_unused((1 << NUM_CORES) - 1, false);
  if (doorbell_counter == -1) {
    char msg[] = "doorbell claim is failed!";
    hal_write(1, msg, sizeof(msg));
    exit(1);
  }
  
}

void hal_init_core1(void)
{
  doorbell_irq = multicore_doorbell_irq_num(doorbell_counter);
  irq_set_exclusive_handler(doorbell_irq, alarm_irq_core1);
  irq_set_enabled(doorbell_irq, true);
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
  }
  

  __wfi();
}

#else
void hal_init(void)
{
  // If this can't get a spinlock, this causes panic.
  mrbc_monitor.vm_mutex = spin_lock_init(spin_lock_claim_unused(true));
  for (int i = 0; i < MUTEX_REQUIRE_NUM; i++) {
    mrbc_monitor.is_available[i] = true;
  }
}

#endif /* ifndef MRBC_NO_TIMER */

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
  
  vm_mutex_lock( WRITE_MUTEX );

  while (--i >= 0)
  {
    putchar(*p++);
    // uart_putc_raw(uart0, *p++ );
  }
  
  vm_mutex_unlock( WRITE_MUTEX );
  
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
