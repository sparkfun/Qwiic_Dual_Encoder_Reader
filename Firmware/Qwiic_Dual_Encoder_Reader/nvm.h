//Location in EEPROM for each thing we want to store between power cycles
enum eepromLocations {
  LOCATION_I2C_ADDRESS, //Device's address
  LOCATION_INTERRUPTS,
  LOCATION_RED_BRIGHTNESS,
  LOCATION_GREEN_BRIGHTNESS,
  LOCATION_BLUE_BRIGHTNESS,
  LOCATION_RED_CONNECT_AMOUNT,
  LOCATION_GREEN_CONNECT_AMOUNT = LOCATION_RED_CONNECT_AMOUNT + 2, //Previous was an int
  LOCATION_BLUE_CONNECT_AMOUNT = LOCATION_GREEN_CONNECT_AMOUNT + 2, //Previous was an int
  LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT = LOCATION_BLUE_CONNECT_AMOUNT + 2, //Previous was an int
  LOCATION_ROTATION_LIMIT = LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT + 2, // Previous was an int
};

//Defaults for the I2C address
const byte I2C_ADDRESS_DEFAULT = 0x3F;
const byte I2C_FORCED_ADDRESS = 0x3E; //This is the address we go to incase user closes the address jumper
