#include "subbus.h"
#include "rtc_timer.h"
#include "control.h"

/*
const int ibufsize = 80;
char ibuf[ibufsize];
int nc, cp;
uint32_t nloop;

uint32_t timeout;
*/

void setup() {
  if (subbus_add_driver(&sb_base) ||
      subbus_add_driver(&sb_fail_sw) ||
      subbus_add_driver(&sb_board_desc) ||
      subbus_add_driver(&sb_control) ||
      subbus_add_driver(&sb_rtc)
      ) {
    Serial.println("Misconfigured subbus driver");
  }
  subbus_reset();
}

/*
bool expired(uint32_t endtime) {
  return (rtc_current_count - endtime) < 0x80000000U;
}
*/

void loop() {
  subbus_poll();
  /*
  ++nloop;
  if (expired(timeout)) {
    int avW = Serial.availableForWrite();
    Serial.print("AvailableForWrite = ");
    Serial.println(avW);
    Serial.println("Timeout!");
    timeout += 1000*RTC_COUNTS_PER_MSEC;
  }
  int rv = nc < ibufsize ?
    Serial.readBytes(ibuf+nc, ibufsize-nc) : 0;
  if (rv > 0) {
    Serial.print("nloop=");
    Serial.print(nloop);
    Serial.print(" time=");
    Serial.print(rtc_current_count);
    Serial.print(" rv=");
    Serial.println(rv);
  }
  nc += rv;
  */
}
