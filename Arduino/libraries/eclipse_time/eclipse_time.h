#ifndef __ECLIPSE_TIME_INCLUDED__
#define __ECLIPSE_TIME_INCLUDED__

#include <TinyGPS++.h>

class eclipse_time {

public:
  eclipse_time();
  eclipse_time(const eclipse_time &from);
  void set(const TinyGPSPlus &gps);
  void set_base_millis(uint32_t  base_millis_lhs) {
    base_millis = base_millis_lhs;  
  }

  void set_timestamp_millis(uint32_t timestamp_millis_lhs) {
    timestamp_millis = timestamp_millis_lhs;
  }

  uint16_t get_year() const {
    return year;
  }

  uint8_t get_month() const {
    return month;
  }

  uint8_t get_hour() const {
    return hour;
  }

  uint8_t get_day() const {
    return day;
  }

  uint8_t get_minute() const {
    return minute;
  }

  uint8_t get_second() const {
    return second;
  }

  uint32_t get_base_millis() const {
    return base_millis;
  }

  uint32_t get_timestamp_millis() const {
    return timestamp_millis;
  }

private:
  uint32_t base_millis;   // Milliseconds at the second before last update
  uint32_t timestamp_millis;  // Milliseconds at timestamp request
  uint16_t year;
  uint8_t month;
  uint8_t day;

  uint8_t hour;
  uint8_t minute;
  uint8_t second; 
};
#endif // __ECLIPSE_TIME_INCLUDED__

