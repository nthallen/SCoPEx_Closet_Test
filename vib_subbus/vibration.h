#ifndef VIBRATION_H_INCLUDED
#define VIBRATION_H_INCLUDED
#include <DPEng_ICM20948_AK09916.h>
#include "subbus.h"

#define VIB_BASE_ADDR 0x50
#define VIB_CMD_OFFSET 0
#define VIB_STATUS_OFFSET 0
#define VIB_SAMPLE_DUR_OFFSET 1
#define VIB_PREQDEPTH_OFFSET 2
#define VIB_FIFODEPTH_OFFSET 3
#define VIB_FIFO_OFFSET 4
#define VIB_TDELTA_OFFSET 5
#define VIB_HIGH_ADDR (VIB_BASE_ADDR+VIB_TDELTA_OFFSET)

#define VIB_STAT_SENSOR_RDY 0x01
#define VIB_STAT_FIFO_OVFLOW 0x02
#define VIB_STAT_DUPLICATE 0x04
#define VIB_STAT_RESET 0x08
#define VIB_STAT_PRESET 0x10
#define VIB_STAT_ACTIVE 0x20
#define VIB_STAT_STATES (VIB_STAT_RESET|VIB_STAT_PRESET|VIB_STAT_ACTIVE)

#define icm_odper 2000

extern subbus_driver_t sb_vib;

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t dT;
} vib_fifo_record;
#define VIB_FIFO_SIZE 20

#endif
