#ifndef _EEPROMhandler_h
#define _EEPROMhandler_h

unsigned char EEPROM_read(unsigned int uiAddress){
  while(EECR & (1<<EEPE));  
    EEAR = uiAddress;
    EECR |= (1<<EERE);
    return EEDR;
}
 
void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
  while(EECR & (1<<EEPE)); 
    EEAR = uiAddress; 
    EEDR = ucData; 
    EECR |= (1<<EEMPE);
    EECR |= (1<<EEPE); 
} 

void EEPROM_get(unsigned int uiAddress, char *buffer, int n){

  unsigned int i = 0;
  do {
    buffer[i] = EEPROM_read(uiAddress + i);
    i += 1;

  } while(i < n && buffer[i] != 0xff);
  buffer[n] = '\0';

}

void EEPROM_put(unsigned int uiAddress, char *buffer, int n){

  unsigned int i = 0;
  do {
    EEPROM_write(uiAddress + i, buffer[i]);
    i += 1;

  } while(i < n);

}
#endif