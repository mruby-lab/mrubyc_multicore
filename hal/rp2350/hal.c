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
spin_lock_t * alloc_mutex;
spin_lock_t * write_mutex;
spin_lock_t * gc_mutex;
spin_lock_t * globalvar_mutex;
spin_lock_t * symbol_mutex;

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
  if (is_busy[get_procid()]) {
    mrbc_tick();
  }
  
  return true;
}



void alarm_irq_at_sleep(void)
{
  uint8_t procid = get_procid();
  if (is_busy[procid] == false) {
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
                        | CLOCKS_SLEEP_EN1_CLK_USB_BITS
                        | CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS
                        | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS;
  
  alloc_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  write_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  gc_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  globalvar_mutex = vm_mutex_init(spin_lock_claim_unused(false));
  symbol_mutex = vm_mutex_init(spin_lock_claim_unused(false));
}

void hal_init_core1(void)
{
  alarm_pool_t * pool = alarm_pool_create_with_unused_hardware_alarm(true);
  alarm_pool_add_repeating_timer_ms(pool, 1, alarm_irq, NULL, &timer1);
}

//================================================================
/*!@brief
  Flush write buffer

  @param  fd    dummy, but 1.
*/
int hal_flush(int fd) {
  return 0;
}

void goto_sleep_for_1ms()
{
  struct timespec ts;
  uint8_t procid = get_procid();
  is_busy[procid] = false;
  aon_timer_get_time(&ts);
  
  ts.tv_nsec += 1e6;
  if (ts.tv_nsec >= 1e9) {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1e9;
  }

  aon_timer_enable_alarm(&ts, alarm_irq_at_sleep, true);

  __wfi();
}

#else
void hal_init(void){
  alloc_mutex = vm_mutex_init(spin_lock_claim_unused(true));
  write_mutex = vm_mutex_init(spin_lock_claim_unused(true));
  gc_mutex = vm_mutex_init(spin_lock_claim_unused(true)); 
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
  interrupt_status_t save;

  save = vm_mutex_lock(write_mutex);

  while( --i >= 0 ) {
    putchar( *p++ );
    // uart_putc_raw(uart0, *p++ );
  }

  vm_mutex_unlock(write_mutex, save);

  return nbytes;
}


//================================================================
/*!@brief
  abort program

  @param s	additional message.
*/
void hal_abort(const char *s)
{
  if( s ) {
    hal_write(1, s, strlen(s));
  }

  abort();
}
