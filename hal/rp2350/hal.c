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

static uint8_t is_busy[2] = {true, true};

/***** Global variables *****************************************************/
uint32_t monitor_pre_canary[1];
Monitor mrbc_monitor;
uint32_t mrbc_monitor_post_canary[1];

/***** Signal catching functions ********************************************/
/***** Local functions ******************************************************/

/***** Global functions *****************************************************/
#ifndef MRBC_NO_TIMER

//================================================================
/*!@brief
  timer alarm irq

*/
bool alarm_irq(struct repeating_timer *t)
{
  if (is_busy[get_procid()])
  {
    mrbc_tick();
  }

  return true;
}

void alarm_irq_at_sleep(void)
{
  int8_t procid = get_procid();
  if (is_busy[procid] == false)
  {
    mrbc_tick();
  }
  is_busy[procid] = true;
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
    mrbc_monitor.owner[i] = -1;
    mrbc_monitor.change_seq[i] = 0u;
  }
  
  gpio_init(OUT_PIN);
  gpio_set_dir(OUT_PIN, GPIO_OUT);
  gpio_put(OUT_PIN, 0);
  
  monitor_pre_canary[0] = CANARY_VAL;

  mrbc_monitor_post_canary[0] = CANARY_VAL;
}

void hal_init_core1(void)
{
  alarm_pool_t *pool = alarm_pool_create_with_unused_hardware_alarm(true);
  alarm_pool_add_repeating_timer_ms(pool, 1, alarm_irq, NULL, &timer1);
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

void goto_sleep_for_1ms()
{
  struct timespec ts;
  int8_t procid = get_procid();
  is_busy[procid] = false;
  aon_timer_get_time(&ts);

  ts.tv_nsec += 1e6;
  if (ts.tv_nsec >= 1e9)
  {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1e9;
  }

  aon_timer_enable_alarm(&ts, alarm_irq_at_sleep, true);

  __wfi();
}

#else
void hal_init(void)
{
  // If this can't get a spinlock, this causes panic.
  mrbc_monitor.vm_mutex = spin_lock_init(spin_lock_claim_unused(true));
  mrbc_monitor.vm_mutex = spin_lock_init(spin_lock_claim_unused(true));
  for (int i = 0; i < MUTEX_REQUIRE_NUM; i++) {
    mrbc_monitor.is_available[i] = true;
    mrbc_monitor.owner[i] = -1;
    mrbc_monitor.change_seq[i] = 0u;
  }
  
  gpio_init(OUT_PIN);
  gpio_set_dir(OUT_PIN, GPIO_OUT);
  gpio_put(OUT_PIN, 0);
  
  monitor_pre_canary[0] = CANARY_VAL;

  mrbc_monitor_post_canary[0] = CANARY_VAL;
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

  // putchar(mrbc_monitor.is_available[WRITE_MUTEX] + 'a');
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

void check_canary(void) {
  if (monitor_pre_canary[0] != CANARY_VAL || mrbc_monitor_post_canary[0] != CANARY_VAL) {  
    // CANARY 崩壊：メモリ破壊の疑い
    gpio_put(OUT_PIN, 1);
    // ここで安全に止める／リセットするかログを残す
    assert(monitor_pre_canary[0] == CANARY_VAL);
    assert(mrbc_monitor_post_canary[0] == CANARY_VAL);
  }
}



void vm_mutex_lock(const int resource)
{
  interrupt_status_t save;
  while (true) {
    save = spin_lock_blocking(mrbc_monitor.vm_mutex);
    if (mrbc_monitor.is_available[resource]) {
      mrbc_monitor.is_available[resource] = 0;
      mrbc_monitor.change_seq[resource]++;
      mrbc_monitor.owner[resource] = get_procid();
      spin_unlock(mrbc_monitor.vm_mutex, save);
      break;
    }
    spin_unlock(mrbc_monitor.vm_mutex, save);
    tight_loop_contents();
  }
}

void vm_mutex_unlock(const int resource)
{
  
  interrupt_status_t save;
  
  save = spin_lock_blocking(mrbc_monitor.vm_mutex);
  mrbc_monitor.is_available[resource] = 1;
  mrbc_monitor.owner[resource] = -1;
  spin_unlock(mrbc_monitor.vm_mutex, save);
}
