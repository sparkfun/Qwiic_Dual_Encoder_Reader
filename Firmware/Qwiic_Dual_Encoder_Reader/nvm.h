//Location in EEPROM for each thing we want to store between power cycles
enum eepromLocations {
  LOCATION_I2C_ADDRESS, //Device's address
  LOCATION_INTERRUPTS,
  LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT,
  LOCATION_ROTATION_LIMIT = LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT + 2, // Previous was an int
};

//Defaults for the I2C address
const byte I2C_ADDRESS_DEFAULT = 0x73;
const byte I2C_FORCED_ADDRESS = 0x74; //This is the address we go to incase user closes the address jumper
