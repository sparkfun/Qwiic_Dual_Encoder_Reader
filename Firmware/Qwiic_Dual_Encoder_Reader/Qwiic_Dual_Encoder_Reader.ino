/*
  An I2C based Dual Encoder Reader
  By: Pete Lewis
  SparkFun Electronics 
  Date: Jan 30th, 2020
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This code was initially based off of the Qwiic Twist rotary encoder project:
  By: Nathan Seidle, SparkFun Electronics, October 27th, 2018
  And then modified again by: Brian Schmalz, May 27th, 2019
  Read all about it here: https://www.sparkfun.com/products/15083

  Qwiic Dual Encoder Reader is an I2C based encoder reader that tracks how much a pair of quadrature encoders
  have turned. This is handy when you want to know how much your wheels have turned on a two-wheel robot, but also 
  useful for any project that might need two separate encoder inputs/sensors.

  This firmware is intened to be used on an ATTiny84, but can also be used on an ATMEGA328 for easier development.

  It is used on the ATTINY84 that lives on the SparkFun Auto pHAT for Raspberry Pi.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15083

  To install support for ATtiny84 in Arduino IDE: https://github.com/SpenceKonde/ATTinyCore/blob/master/Installation.md
  This core is installed from the Board Manager menu
  This core has built in support for I2C S/M and serial
  If you have Dave Mellis' ATtiny installed you may need to remove it from \Users\xx\AppData\Local\Arduino15\Packages

  To support 400kHz I2C communication reliably ATtiny84 needs to run at 8MHz. This requires user to
  click on 'Burn Bootloader' before code is loaded.
*/

#include <Wire.h>
#include <EEPROM.h>
#include "nvm.h"

#include "PinChangeInterrupt.h" //Nico Hood's library: https://github.com/NicoHood/PinChangeInterrupt/
//Used for pin change interrupts on ATtinys (encoder turns cause interrupt)
//Note: To make this code work with Nico's library you have to comment out https://github.com/NicoHood/PinChangeInterrupt/blob/master/src/PinChangeInterruptSettings.h#L228

#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers
#include <avr/io.h>    //Needed for reading the entire port in one instruction, for faster reading of all encoder pins

#if defined(__AVR_ATmega328P__)
//Hardware connections while developing with an Uno
const byte addressPin = 6;
const byte encoder2BPin = 5; 
const byte encoder2APin = 4;
const byte encoder1BPin = 3;
const byte encoder1APin = 2;
const byte interruptPin = 7;

#elif defined(__AVR_ATtiny84__)
//Hardware connections for the final design
const byte addressPin = 9;
const byte encoder2BPin = 8;
const byte encoder2APin = 7;
const byte encoder1BPin = 2;
const byte encoder1APin = 10;
const byte interruptPin = 0;
#endif

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Variables used in the I2C interrupt so we use volatile

//This is the pseudo register map of the product. If user asks for 0x02 then get the 3rd
//byte inside the register map.
struct memoryMap {
  byte id;
  byte status;
  byte firmwareMajor;
  byte firmwareMinor;
  byte interruptEnable;
  int16_t encoder1Count;
  int16_t encoder1Difference;
  int16_t encoder2Count;
  int16_t encoder2Difference;  
  uint16_t timeSinceLastMovement;
  uint16_t turnInterruptTimeout;
  byte i2cAddress;
  uint16_t rotationLimit;
};

const byte statusEncoderMovedBit = 0;
const byte enableInterruptEncoderBit = 0;

//These are the defaults for all settings
volatile memoryMap registerMap = {
  .id =                         0x5C, // 0x00  
  .status =                     0x00, // 0x01         0 - encoder moved
  .firmwareMajor =              0x01, // 0x02         Firmware version. Helpful for tech support.
  .firmwareMinor =              0x02, // 0x03
  .interruptEnable =            0x03, // 0x04         0 - encoder interrupt
  .encoder1Count =             0x0000, // 0x05, 0x06
  .encoder1Difference =        0x0000, // 0x07, 0x08
  .encoder2Count =             0x0000, // 0x09, 0x0A
  .encoder2Difference =        0x0000, // 0x0B, 0x0C  
  .timeSinceLastMovement =    0x0000, // 0x0D, 0x0E
  .turnInterruptTimeout =        250, // 0x0F, 0x10
  .i2cAddress =  I2C_ADDRESS_DEFAULT, // 0x11
  .rotationLimit =                 0, // 0x12, 0x13   0 means disable this feature (disabled by default)
};

//This defines which of the registers are read-only (0) vs read-write (1)
memoryMap protectionMap = {
  .id = 0x00,
  .status = (1 << statusEncoderMovedBit), //0 - encoder moved
  .firmwareMajor = 0x00,
  .firmwareMinor = 0x00,
  .interruptEnable = (1 << enableInterruptEncoderBit), //1 - button int enable, 0 - encoder int enable
  .encoder1Count = (int16_t)0xFFFF,
  .encoder1Difference = (int16_t)0xFFFF,
  .encoder2Count = (int16_t)0xFFFF,
  .encoder2Difference = (int16_t)0xFFFF,  
  .timeSinceLastMovement = (uint16_t)0xFFFF,
  .turnInterruptTimeout = (uint16_t)0xFFFF,
  .i2cAddress = 0xFF,
  .rotationLimit = 0xFFFF,
};

//Cast 32bit address of the object registerMap with uint8_t so we can increment the pointer
uint8_t *registerPointer = (uint8_t *)&registerMap;
uint8_t *protectionPointer = (uint8_t *)&protectionMap;

volatile byte registerNumber; //Gets set when user writes an address. We then serve the spot the user requested.

volatile boolean updateOutputs = false; //Goes true when we receive new bytes from user. Causes things to update in main loop.

volatile byte lastEncoded1 = 0; //Used to compare encoder readings between interrupts. Helps detect turn direction.

volatile byte lastEncoded2 = 0; //Used to compare encoder readings between interrupts. Helps detect turn direction.

volatile unsigned long lastEncoderMoveTime; //Time stamp of last encoder movement

//Interrupt turns on when encoder is moved
//turns off when interrupts are cleared by command
enum State {
  STATE_ENCODER_INT = 0,
  STATE_INT_CLEARED,
  STATE_INT_INDICATED,
};
volatile byte interruptState = STATE_INT_CLEARED;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup(void)
{
  pinMode(addressPin, INPUT_PULLUP);
  pinMode(encoder2BPin, INPUT); //No pull-up. External 10k
  pinMode(encoder2APin, INPUT); //No pull-up. External 10k
  pinMode(encoder1BPin, INPUT); //No pull-up. External 10k
  pinMode(encoder1APin, INPUT); //No pull-up. External 10k
  pinMode(interruptPin, INPUT); //Interrupt is high-impedance until we have int (and then go low). Pulled high with 10k with cuttable jumper.

#if defined(__AVR_ATmega328P__)
  pinMode(addressPin, INPUT_PULLUP);
  pinMode(encoder2BPin, INPUT_PULLUP);
  pinMode(encoder2APin, INPUT_PULLUP);
  pinMode(encoder1BPin, INPUT_PULLUP);
  pinMode(encoder1APin, INPUT_PULLUP);
  pinMode(interruptPin, INPUT); //Interrupt is high-impedance until we have int (and then go low). Optional external pull up.
#endif

  turnOffExtraBits(); //Turn off all unused peripherals

  readSystemSettings(); //Load all system settings from EEPROM

  setupInterrupts(); //Enable pin change interrupts for I2C, encoder, switch, etc

  lastEncoderMoveTime = 0; //User has not yet twisted the encoder. Used for firing int pin.

  startI2C(); //Determine the I2C address we should be using and begin listening on I2C bus

#if defined(__AVR_ATmega328P__)
  Serial.begin(9600);
  Serial.println("Qwiic Dual Encoder Reader");
  Serial.print("Address: 0x");

  if (digitalRead(addressPin) == HIGH) //Default is HIGH, the address jumper is open
    Serial.print(registerMap.i2cAddress, HEX);
  else
    Serial.print(I2C_FORCED_ADDRESS, HEX);
  Serial.println();

#endif
}

void loop(void)
{
  //Interrupt pin state machine
  //There are four states: Encoder Int, Int Cleared, Int Indicated
  //ENCODER_INT state is set here once user has stopped turning encoder
  //INT_CLEARED state is set in the I2C interrupt when Clear Ints command is received.
  //INT_INDICATED state is set once we change the INT pin to go low
  if (interruptState == STATE_INT_CLEARED)
  {
    //See if user has turned the encoder
    if ( registerMap.status & (1 << statusEncoderMovedBit) )
    {
      //Check if encoder interrupt is enabled
      if ( (registerMap.interruptEnable & (1 << enableInterruptEncoderBit) ) )
      {
        //See if enough time has passed since the user has stopped turning the encoder
        if ( (millis() - lastEncoderMoveTime) > registerMap.turnInterruptTimeout)
        {
          interruptState = STATE_ENCODER_INT; //Go to next state
        }
      }
    }
  }

  //If we are in encoder state, then set INT low
  if(interruptState == STATE_ENCODER_INT)
  {
    //Set the interrupt pin low to indicate interrupt
    pinMode(interruptPin, OUTPUT);
    digitalWrite(interruptPin, LOW);
    interruptState = STATE_INT_INDICATED;
  }

  if (updateOutputs == true)
  {
    updateOutputs = false;

    //Record anything new to EEPROM
    //It can take ~3.4ms to write EEPROM byte so we do that here instead of in interrupt
    recordSystemSettings();
  }

#if defined(__AVR_ATmega328P__)
  Serial.print("Encoder1: ");
  Serial.print(registerMap.encoder1Count);

  Serial.print("\tDiff1: ");
  Serial.print(registerMap.encoder1Difference);

  Serial.print("\tEncoder2: ");
  Serial.print(registerMap.encoder2Count);

  Serial.print("\tDiff2: ");
  Serial.print(registerMap.encoder2Difference);  

  Serial.print("\tReg: ");
  Serial.print(registerNumber);

  Serial.println();
#endif

  sleep_mode(); //Stop everything and go to sleep. Wake up from Encoder, Button, or I2C interrupts.
}

//If the current setting is different from that in EEPROM, update EEPROM
void recordSystemSettings(void)
{
  //I2C address is byte
  byte i2cAddr;

  //Error check the current I2C address
  if (registerMap.i2cAddress < 0x08 || registerMap.i2cAddress > 0x77)
  {
    //User has set the address out of range
    //Go back to defaults
    registerMap.i2cAddress = I2C_ADDRESS_DEFAULT;
  }

  //Read the value currently in EEPROM. If it's different from the memory map then record the memory map value to EEPROM.
  EEPROM.get(LOCATION_I2C_ADDRESS, i2cAddr);
  if (i2cAddr != registerMap.i2cAddress)
  {
    EEPROM.put(LOCATION_I2C_ADDRESS, (byte)registerMap.i2cAddress);
    startI2C(); //Determine the I2C address we should be using and begin listening on I2C bus
  }

  byte intBits;
  EEPROM.get(LOCATION_INTERRUPTS, intBits);
  if (intBits != registerMap.interruptEnable)
    EEPROM.put(LOCATION_INTERRUPTS, (byte)registerMap.interruptEnable);


  //Turn Timeout is uint16_t
  uint16_t timeout;

  EEPROM.get(LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT, timeout);
  if (timeout != registerMap.turnInterruptTimeout)
    EEPROM.put(LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT, (int16_t)registerMap.turnInterruptTimeout);

  //If the user has zero'd out the timestamps then reflect that in the globals
  if (registerMap.timeSinceLastMovement == 0) lastEncoderMoveTime = 0;

  //Rotation Limit is uint16_t
  uint16_t rotationLim;

  EEPROM.get(LOCATION_ROTATION_LIMIT, rotationLim);
  if (rotationLim != registerMap.rotationLimit)
    EEPROM.put(LOCATION_ROTATION_LIMIT, (int16_t)registerMap.rotationLimit);
}

//Reads the current system settings from EEPROM
//If anything looks weird, reset setting to default value
void readSystemSettings(void)
{
  uint16_t discard; // do-nothing variable put in to eleminate compiler warnings

  //Read what I2C address we should use
  registerMap.i2cAddress = EEPROM.read(LOCATION_I2C_ADDRESS);
  if (registerMap.i2cAddress == 0xFF) //Blank
  {
    registerMap.i2cAddress = I2C_ADDRESS_DEFAULT; //By default, we listen for I2C_ADDRESS_DEFAULT
    EEPROM.write(LOCATION_I2C_ADDRESS, registerMap.i2cAddress);
  }

  //Error check I2C address we read from EEPROM
  if (registerMap.i2cAddress < 0x08 || registerMap.i2cAddress > 0x77)
  {
    //User has set the address out of range
    //Go back to defaults
    registerMap.i2cAddress = I2C_ADDRESS_DEFAULT;
    EEPROM.write(LOCATION_I2C_ADDRESS, registerMap.i2cAddress);
  }

  //Read the interrupt bits
  registerMap.interruptEnable = EEPROM.read(LOCATION_INTERRUPTS);
  if (registerMap.interruptEnable == 0xFF) //Blank
  {
    registerMap.interruptEnable = 0x03; //By default, enable the click and encoder interrupts
    EEPROM.write(LOCATION_INTERRUPTS, registerMap.interruptEnable);
  }


  discard = EEPROM.get(LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT, registerMap.turnInterruptTimeout);
  if ((uint16_t)registerMap.turnInterruptTimeout == 0xFFFF)
  {
    registerMap.turnInterruptTimeout = 250; //Default to 250ms
    EEPROM.put(LOCATION_TURN_INTERRUPT_TIMEOUT_AMOUNT, (int16_t)registerMap.turnInterruptTimeout);
  }

  discard = EEPROM.get(LOCATION_ROTATION_LIMIT, registerMap.rotationLimit);
  if ((uint16_t)registerMap.rotationLimit == 0xFFFF)
  {
    registerMap.rotationLimit = 0; // Default to 0 (off)
    EEPROM.put(LOCATION_ROTATION_LIMIT, (int16_t)registerMap.rotationLimit);
  }

  // To eleminate compiler warning
  (void)discard;
}

//Turn off anything we aren't going to use
void turnOffExtraBits()
{
  //Disable ADC
  ADCSRA = 0;

  //Disble Brown-Out Detect
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);

  //Power down various bits of hardware to lower power usage
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
}

//Begin listening on I2C bus as I2C slave using the global variable setting_i2c_address
void startI2C()
{
  Wire.end(); //Before we can change addresses we need to stop

  if (digitalRead(addressPin) == HIGH) //Default is HIGH, the address jumper is open
    Wire.begin(registerMap.i2cAddress); //Start I2C and answer calls using address from EEPROM
  else
    Wire.begin(I2C_FORCED_ADDRESS); //Force address to I2C_FORCED_ADDRESS if user has closed the solder jumper

  //The connections to the interrupts are severed when a Wire.begin occurs. So re-declare them.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}
