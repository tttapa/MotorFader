// Disable interrupts first
void EEPROM_write(unsigned char ucAddress, unsigned char ucData) {
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set Programming mode */
EECR = (0<<EEPM1)|(0<<EEPM0);
/* Set up address and data registers */
EEAR = ucAddress;
EEDR = ucData;
/* Write logical one to EEMPE */
EECR |= (1<<EEMPE);
/* Start eeprom write by setting EEPE */
EECR |= (1<<EEPE);
}

// Disable interrupts first
unsigned char EEPROM_read(unsigned char address) {
  /* Wait for completion of previous write */
  while (EECR & (1 << EEPE));
  /* Set up address register */
  EEAR = address;
  /* Start eeprom read by writing EERE */
  EECR |= (1 << EERE);
  /* Return data from data register */
  return EEDR;
}

int main() {
  cli();
  EEPROM_write(0, 42);
  EEPROM_read(0);
}

