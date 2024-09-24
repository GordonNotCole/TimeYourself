#include "EEPROMhandler.h"
#include "TinyGps-submodule/src/TinyGPS.h"
#include "UARThandler.h" 
#include "RingBuf.h"

#define EEPROM_INF 0
#define EEPROM_LAT 10
#define EEPROM_LNG 20
#define EEPROM_DATA_S 31
#define EEPROM_TIME_S 35
#define EEPROM_DATA_E 40
#define EEPROM_TIME_E 45
#define EEPROM_STEP 50
#define EEPROM_LAST_POS 1023

#define THRESHOLD_TIME 300 // seconds for a new place
#define THRESHOLD_DIST 50 // meters

int32_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
int32_t term_time = 0, wait_location = 0, term_flag = 0;
double term_lng = 0, term_lat = 0;
uint8_t term_sec, term_minute, term_hour, term_day, term_month;
uint16_t term_year;

void setup(void) { 

	    init_UART();
      recover();
      get_info();

	
}

int32_t prefix_mas(int32_t *mas, int32_t index){

      int32_t res = 0;
      for(size_t i = 0; i < index; i++){
            res += mas[i];
      }
      return res;
}

int32_t f_to_s(double value, int32_t*buffer){

      value *= 10000000;
      unsigned long valuev = value;
      int32_t i;
      for(i = 0; valuev != 0; i++){
            buffer[i] = valuev % 10;
            valuev /= 10;
      }
      return i;
}

int32_t i_to_s(int32_t value, int32_t* buffer){

      int32_t i;
      for(i = 0; value != 0; i++){
            buffer[i] = value % 10;
            value /= 10;
      }
      return i;
}

void f_to_s_e(double value, unsigned int addr){

      int32_t buffer[20];
      int32_t j = f_to_s(value, buffer) - 1;
      uint32_t i = 0, val_zero = 0;

      for(;j >= 0;){
            i += 1;
            if((int32_t)value == 0 && val_zero == 0){
                  EEPROM_write(addr + i, '.');
                  val_zero = 1;
            } else {
                  EEPROM_write(addr + i, buffer[j] + '0');
                  j -= 1;
            }
            value /= 10;
      }

      EEPROM_write(addr + i, '\0');

}

void to_buf(char *buffer){
  
      for(size_t j = 0;  buffer[j] != '\0'; j += 1){
            buf_write.write(buffer[j]);
      }
      send();

}

void write_date_time(TinyGPSPlus gps, unsigned int addr1, unsigned int addr2, uint32_t i, int32_t term){

      if(term == 0){
            EEPROM_write(i + addr1, gps.date.day());
            EEPROM_write(i + addr1 + 1, gps.date.month());
            EEPROM_write(i + addr1 + 2, gps.date.year() - 1970);

            EEPROM_write(i + addr2, gps.time.hour());
            EEPROM_write(i + addr2 + 1, gps.time.minute());
            EEPROM_write(i + addr2 + 2, gps.time.second());
      }else{
            EEPROM_write(i + addr1, term_day);
            EEPROM_write(i + addr1 + 1, term_month);
            EEPROM_write(i + addr1 + 2, term_year - 1970);

            EEPROM_write(i + addr2, term_hour);
            EEPROM_write(i + addr2 + 1, term_minute);
            EEPROM_write(i + addr2 + 2, term_sec);
      }

}

int32_t set_time(TinyGPSPlus gps){

      int32_t time = (int32_t)gps.date.day() * 24 * 60 * 60 + (int32_t)gps.time.hour() * 60 * 60 + (int32_t)gps.time.minute() * 60 + (int32_t)gps.time.second();

      return time;

}

int32_t absolute_time(TinyGPSPlus gps){

      if(abs(set_time(gps) - term_time) <= THRESHOLD_TIME){
            return 0;
      }
      return 1;

}

void save_location(TinyGPSPlus gps, int32_t pos){

      uint32_t last_pos = (uint32_t)pos;
      uint32_t i = last_pos * EEPROM_STEP;

      term_time = set_time(gps);
      term_lat = gps.location.lat();
      term_lng = gps.location.lng();

      write_date_time(gps, EEPROM_DATA_E, EEPROM_TIME_E, i, 0);

}

uint32_t new_location(TinyGPSPlus gps, int32_t pos){

      uint32_t last_pos = (uint32_t)pos;
      uint32_t i = last_pos * EEPROM_STEP;
      
      if(term_lat == 0 || term_lng == 0 || term_time == 0) 
            return 0;

      write_date_time(gps, EEPROM_DATA_S, EEPROM_TIME_S, i, 1);

      f_to_s_e(term_lat, i + EEPROM_LAT);
      f_to_s_e(term_lng, i + EEPROM_LNG);

      EEPROM_write(EEPROM_LAST_POS, last_pos);

      return 1;

}

int32_t compare(TinyGPSPlus gps){

      char buffer[20];
      double delta;
      uint32_t last_pos, i;
      int32_t pos = 0;

      last_pos = EEPROM_read(EEPROM_LAST_POS);
      i = last_pos * EEPROM_STEP;

      if(last_pos == 255){
            if(term_lat == 0 && term_lng == 0){
                  term_lat = gps.location.lat();
                  term_lng = gps.location.lng();
                  term_time = set_time(gps);
            }
            pos = -1;
      } else {
            pos = last_pos;
      }

      if(pos >= 20)
           return 0;

      delta = gps.distanceBetween(term_lat, term_lng, gps.location.lat(), gps.location.lng());

      if(wait_location == 0 && pos != -1){

                  // int32_t str[20];
                  // char str_str[20];
                  // int32_t index;

                  // // index = f_to_s(gps.location.lat(), str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i < index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n");

                  // index = f_to_s(gps.location.lng(), str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i < index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n");
                  // to_buf("Compare : \n");
                  // index = f_to_s(term_lat, str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i < index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n");

                  // index = f_to_s(term_lng, str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i < index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n\n\n");

                  // if(delta > 10000 && delta < 15000)
                  //     to_buf("yes\n");
                  // index = f_to_s(delta / 1000, str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i <= index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n\n\n");

                  // int32_t delta_int = (int32_t)delta;
                  // index = i_to_s(delta_int, str) - 1;
                  // str_str[index] = '\0'; 
                  // for(int32_t i = 0; i <= index; i++){
                  //       str_str[i] = str[index - i] + '0';
                  // }
                  // to_buf(str_str);
                  // to_buf("\n");

            if(delta >= THRESHOLD_DIST){
                  save_location(gps, pos);
                  wait_location = 1;
            }

      }else{

            if(term_flag == 0){
                  term_day = gps.date.day();
                  term_month =  gps.date.month();
                  term_year = gps.date.year();

                  term_hour = gps.time.hour();
                  term_minute = gps.time.minute();
                  term_sec = gps.time.second(); 

                  term_flag = 1;
            }

            if(!absolute_time(gps))
                 return 0;
            
            term_time = set_time(gps);
            term_flag = 0;

            if(delta <= THRESHOLD_DIST + 5){
                  new_location(gps, pos + 1);
                  wait_location = 0;
            }

            term_lat = gps.location.lat();
            term_lng = gps.location.lng();

      }

}

int32_t recover(){

      uint32_t last_pos = EEPROM_read(EEPROM_LAST_POS);
      int32_t i = last_pos * EEPROM_STEP;
      char day = EEPROM_read(i + EEPROM_DATA_E);
      char str[30];

      term_lat = 0;
      term_lng = 0;
      term_time = 0;
      wait_location = 0;
      term_flag = 0;

      if(last_pos == 255)
            return 0;

      if(((day % 10) + '0') == 47){
            wait_location = 0;
      }else{
            wait_location = 1;
      }

      EEPROM_get(i + EEPROM_LAT + 1, str, 30);
      term_lat = atof(str);

      EEPROM_get(i + EEPROM_LNG + 1, str, 30);
      term_lng = atof(str);

      return 0;

}

void print_date(char day, char month, char year, int32_t tire){

      char str[50];
      int32_t j, k;
      int32_t tmp[10];

      str[0] = (day / 10) % 10 + '0';
      str[1] = (day) % 10 + '0';
      str[2] = '/';
      str[3] = (month / 10) % 10 + '0';
      str[4] = month % 10 + '0';
      str[5] = '/';
            
      j = i_to_s(year + 1970, tmp) - 1;
      k = 6;
      for(; j >= 0; j--, k++){
            str[k] = tmp[j] + '0'; 
      }
      if(tire){
            str[k] = ' ';
            str[k + 1] = ' ';
            str[k + 2] = '-';
            str[k + 3] = ' ';
            str[k + 4] = ' ';
            str[k + 5] = '\0';
      }else{
            str[k] = '\0';
      }
      to_buf(str);

}

void print_time(char hour, char minute, char sec, int32_t tire){

      char str[50];
      int32_t k = 8;

      str[0] = ((hour + 3) / 10) % 10 + '0';
      str[1] = (hour + 3) % 10 + '0';
      str[2] = ':';
      str[3] = (minute / 10) % 10 + '0';
      str[4] = minute % 10 + '0';
      str[5] = ':';
      str[6] = (sec / 10) % 10 + '0';
      str[7] = (sec) % 10 + '0';
      if(tire){
            str[k] = ' ';
            str[k + 1] = ' ';
            str[k + 2] = '-';
            str[k + 3] = ' ';
            str[k + 4] = ' ';
            str[k + 5] = '\0';
      }else{
            str[k] = '\0';
      }
      to_buf(str);
}

void get_info(){

      char buffer[100];
      char str[50];
      int32_t tmp[10];
      int32_t j, k;
      char hour, minute, sec, day, month, year;
      char day_end, month_end, year_end, hour_end, minute_end, sec_end;

      uint32_t last_pos = EEPROM_read(EEPROM_LAST_POS);

      to_buf("---------TABLE---------\n");
    

      if(last_pos == 255)
            return 0;

      for(size_t i = 0; i < (last_pos + 1) * EEPROM_STEP; i += EEPROM_STEP){
        
            to_buf("Latitude : ");
            EEPROM_get(i + EEPROM_LAT, str, 10);
            to_buf(str + 1);
            to_buf("\n");
          
            to_buf("Longitude : ");
            EEPROM_get(i + EEPROM_LNG, str, 10);
            to_buf(str + 1);
            to_buf("\n");

            day = EEPROM_read(i + EEPROM_DATA_S);
            month = EEPROM_read(i + EEPROM_DATA_S + 1);
            year = EEPROM_read(i + EEPROM_DATA_S + 2);

            day_end = EEPROM_read(i + EEPROM_DATA_E);
            month_end = EEPROM_read(i + EEPROM_DATA_E + 1);
            year_end = EEPROM_read(i + EEPROM_DATA_E + 2);

            hour = EEPROM_read(i + EEPROM_TIME_S);
            minute = EEPROM_read(i + EEPROM_TIME_S + 1);
            sec = EEPROM_read(i + EEPROM_TIME_S + 2);

            hour_end = EEPROM_read(i + EEPROM_TIME_E);
            minute_end = EEPROM_read(i + EEPROM_TIME_E + 1);
            sec_end = EEPROM_read(i + EEPROM_TIME_E + 2);


            to_buf("Time : ");
            print_time(hour, minute, sec, 1);
            print_time(hour_end, minute_end, sec_end, 0);
            to_buf("\n");
            

            to_buf("Date : ");
            print_date(day, month, year, 1);
            print_date(day_end, month_end, year_end, 0);
            to_buf("\n\n\n");

            delay(3000);
    }

}

void loop() {
  
      if (gps.encode(receive())) {

            if(gps.date.day() != 0 && gps.date.year() != 1969 && gps.satellites.value() >= 4){

                  compare(gps);
                  get_info();
                  
            }

            delay(5000);

     }
  
}
