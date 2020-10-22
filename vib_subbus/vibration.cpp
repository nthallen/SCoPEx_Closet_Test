/* vibration.cpp */
#include "vibration.h"
#include "rtc_timer.h"

DPEng_ICM20948 dpEng = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

static subbus_cache_word_t vib_cache[VIB_HIGH_ADDR-VIB_BASE_ADDR+1] = {
  { 0, 0, true, false, true,  false, true  }, // cmd/status
  { 0, 0, true, false, true,  false, true  }, // SampleDur
  { 0, 0, true, false, true,  false, false }, // PreQueueDepth
  { 0, 0, true, false, false, false, true  },  // QueueDepth
  { 0, 0, true, false, false, false, true  },  // FIFO
  { 0, 0, true, false, false, false, false }   // Tdelta
};

static uint16_t vib_status = 0;

static vib_update_status(uint16_t mask, uint16_t value) {
  vib_status = (vib_status & ~mask) | (value & mask);
  sb_cache_update(vib_cache, VIB_STATUS_OFFSET, vib_status);
}

static int fifo_head = 0; // Where next entry will go
static int fifo_tail = 0; // Where next read will occur
static uint16_t fifo_depth = 0; // Empty to start
static bool fifo_full = false;
static int fifo_tail_idx = 0; // link into the record
// FIFO is empty if !fifo_full && fifo_head == fifo_tail
// FIFO is full if fifo_full && fifo_head == fifo_tail
static vib_fifo_record vib_fifo[VIB_FIFO_SIZE];

static uint32_t vib_sample_dur = 0; // usecs
static uint32_t vib_next_sample = 0; // usecs

static int16_t last_x;
static int16_t last_y;
static int16_t last_z;

static void push_vib_sample(int16_t x, int16_t y, int16_t z, uint16_t dT) {
  if (fifo_full) {
    vib_update_status(VIB_STAT_FIFO_OVFLOW, VIB_STAT_FIFO_OVFLOW);
    return;
  }
  vib_fifo_record *R = &vib_fifo[fifo_head];
  last_x = R->x = x;
  last_y = R->y = y;
  last_z = R->z = z;
  R->dT = dT;
  if (fifo_head == fifo_tail) {
    sb_cache_update(vib_cache, VIB_FIFO_OFFSET, last_x);
    vib_cache[VIB_FIFODEPTH_OFFSET].readable = true;
    fifo_tail_idx = 0;
  }
  if (++fifo_head == VIB_FIFO_SIZE)
    fifo_head = 0;
  fifo_full = (fifo_head == fifo_tail);
  ++fifo_depth;
  sb_cache_update(vib_cache, VIB_FIFODEPTH_OFFSET,
    fifo_depth*4-fifo_tail_idx);
  vib_next_sample += vib_sample_dur;
}

static void vib_reset_fifo() {
  fifo_head = fifo_tail = fifo_tail_idx = 0;
  fifo_depth = 0;
  sb_cache_update(vib_cache, VIB_FIFODEPTH_OFFSET, 0);
  vib_cache[VIB_FIFODEPTH_OFFSET].readable = false;
}

static void vib_reset() {
  vib_reset_fifo();
  if(dpEng.begin(ICM20948_ACCELRANGE_4G, GYRO_RANGE_250DPS,
          ICM20948_ACCELLOWPASS_473_0_HZ)) {
    vib_update_status(
      VIB_STAT_SENSOR_RDY|VIB_STAT_STATES,
      VIB_STAT_SENSOR_RDY|VIB_STAT_RESET);
  }
}

/**
 * Clear the FIFO, then push nrecs blank records and
 * switch the status to preset
 */
static void vib_preset_fifo(uint16_t nrecs) {
  vib_reset_fifo();
  vib_update_status(
    VIB_STAT_SENSOR_RDY|VIB_STAT_STATES,
    VIB_STAT_SENSOR_RDY|VIB_STAT_PRESET);
  for (int i = 0; i < nrecs; ++i) {
    push_vib_sample(0, 0, 0, 0);
  }
}

static void vib_poll() {
  if (!(vib_status & VIB_STAT_SENSOR_RDY)) {
    vib_reset();
    return;
  }
  if (vib_status & VIB_STAT_ACTIVE) {
    sensors_event_t aevent; //  Accel only
    dpEng.getEventAcc(&aevent);  //  Accel only

    if (dpEng.accel_raw.x != last_x ||
        dpEng.accel_raw.y != last_y ||
        dpEng.accel_raw.z != last_z) {
      // This is a new sample
      if (rtc_current_count >= vib_next_sample) {
        uint16_t dT = rtc_current_count - vib_next_sample;
        push_vib_sample(dpEng.accel_raw.x, dpEng.accel_raw.y,
          dpEng.accel_raw.z, dT);
      }
    } else if (rtc_current_count >= vib_next_sample + 2*icm_odper) {
      // We should have seen a new value by now. push the old one
      push_vib_sample(last_x, last_y, last_z, 0);
      vib_update_status(VIB_STAT_DUPLICATE, VIB_STAT_DUPLICATE);
    }
  }
}

static void vib_action(uint16_t offset) {
  uint16_t val;
  vib_fifo_record *R;
  switch (offset) {
    case VIB_CMD_OFFSET:
      if (sb_cache_iswritten(vib_cache, VIB_CMD_OFFSET, &val)) {
        switch (val) {
          case 0: // Reset
            vib_update_status(VIB_STAT_STATES, VIB_STAT_RESET);
            break;
          case 1: // Preset: prepopulate the FIFO
            vib_preset_fifo(vib_cache[VIB_PREQDEPTH_OFFSET].cache);
            break;
          default:
            break;
        }
      } else if (vib_status|VIB_STAT_FIFO_OVFLOW|VIB_STAT_DUPLICATE) {
        // will assume status was read. Clear transient bits
        vib_update_status(VIB_STAT_FIFO_OVFLOW|VIB_STAT_DUPLICATE, 0);
      }
      break;
    case VIB_SAMPLE_DUR_OFFSET:
      if (sb_cache_iswritten(vib_cache, VIB_SAMPLE_DUR_OFFSET, &val)) {
        vib_sample_dur = val * 100;
      }
      break;
    case VIB_FIFODEPTH_OFFSET:
      // if preset, then switch to active and set vib_next_sample
      if (vib_status & VIB_STAT_PRESET) {
        vib_update_status(VIB_STAT_STATES, VIB_STAT_ACTIVE);
        vib_next_sample = rtc_current_count;
      }
      break;
    case VIB_FIFO_OFFSET:
      // update FIFO contents based on fifo_tail and fifo_tail_idx
      // if fifo is empty, then we should really make it unreadable
      R = &vib_fifo[fifo_tail];
      switch (++fifo_tail_idx) {
        case 1: val = R->y; break;
        case 2: val = R->z; break;
        case 3: val = R->dT; break;
        case 4:
          fifo_tail_idx = 0;
          fifo_full = false;
          if (++fifo_tail == VIB_FIFO_SIZE)
            fifo_tail = 0;
          --fifo_depth;
          if (fifo_tail == fifo_head) {
            vib_cache[VIB_FIFODEPTH_OFFSET].readable = false;
            val = 0;
          } else {
            val = vib_fifo[fifo_tail].x;
          }
          sb_cache_update(vib_cache, VIB_FIFODEPTH_OFFSET,
            fifo_depth*4-fifo_tail_idx);
          break;
      }
      sb_cache_update(vib_cache, VIB_FIFO_OFFSET, val);
      break;
  }
}

subbus_driver_t sb_vib = {
  VIB_BASE_ADDR, VIB_HIGH_ADDR, // address range
  vib_cache,
  vib_reset,
  vib_poll,
  vib_action,
  false
};
