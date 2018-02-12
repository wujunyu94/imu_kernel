#include "hrtimer.h"

MODULE_LICENSE("GPL");

struct spi_device *spi_device;

static struct hrtimer hr_timer;
static ktime_t ktime;

enum hrtimer_restart my_hrtimer_callback( struct hrtimer *timer )
{
  unsigned char rawData[14];
  short data[6];

  readMultiple(ACCEL_XOUT_H,14,rawData);
  data[0] = ((short)rawData[0]  << 8) | rawData[1];
  data[1] = ((short)rawData[2]  << 8) | rawData[3];
  data[2] = ((short)rawData[4]  << 8) | rawData[5];
  data[3] = ((short)rawData[8]  << 8) | rawData[9];
  data[4] = ((short)rawData[10] << 8) | rawData[11];
  data[5] = ((short)rawData[12] << 8) | rawData[13];
  
  printk("ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",data[0],data[1],data[2],data[3],data[4],data[5]);

  hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );

  return HRTIMER_NORESTART;
}
 
int init_module( void )
{
  unsigned long delay_in_ms = 100L;
 
  printk("HR Timer module installing\n");
 
  ktime = ktime_set( 0, MS_TO_NS(delay_in_ms) );
 
  hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
   
  hr_timer.function = &my_hrtimer_callback;
 
  printk( "Starting timer to fire in %ldms (%ld)\n", delay_in_ms, jiffies );

  spi_init();

  if(!whoAmI())
  {
    InitIMU(1);
    hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );
  }
 
  return 0;
}
 
void cleanup_module( void )
{
  int ret;
 
  spi_deinit();

  ret = hrtimer_cancel( &hr_timer );

  if (ret) printk("The timer was still in use...\n");
 
  printk("HR Timer module uninstalling\n");
 
  return;
}

