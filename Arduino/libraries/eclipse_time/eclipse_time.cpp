/**
 * Class that holds eclipse date timestamps.
 */
#include "eclipse_time.h"

eclipse_time::eclipse_time() :
   base_millis(0),
   timestamp_millis(0),
   year(0),
   month(0),
   day(0),
   hour(0),
   minute(0),
   second(0)
   {}

eclipse_time::eclipse_time(const eclipse_time &from) :
   base_millis(from.base_millis),
   timestamp_millis(from.timestamp_millis),
   year(from.year),
   month(from.month),
   day(from.day),
   hour(from.hour),
   minute(from.minute),
   second(from.second)
   {}

void eclipse_time::set(const TinyGPSPlus &gps) {
  base_millis = 0;
  timestamp_millis = 0;
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();

  hour = gps.time.hour();
  minute = gps.time.minute();
  second = gps.time.second();
}


