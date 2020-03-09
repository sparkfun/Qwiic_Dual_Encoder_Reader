//Called every time the encoder is twisted
//Based on http://bildr.org/2012/08/rotary-encoder-arduino/
//Adam - I owe you many beers for bildr
void updateEncoder1() {

  byte MSB1 = 0;
  byte LSB1 = 0;

#if defined(__AVR_ATmega328P__)
  byte fullPortReadValue = PIND;// Read entire port D, encoders are on D2, D3, D4 and D5, so bits: 00?? ??00 (aka PD2, PD3, PD4, PD5)

  // pull out MSB1 and LSM from full port on both encoders.
  // encoder1 is on 2 and 3, so B0000??00
  if (fullPortReadValue & B00000100) MSB1 = 1;
  if (fullPortReadValue & B00001000) LSB1 = 1;

#elif defined(__AVR_ATtiny84__)
  /*
     In order to keep the interrupts and pin selections (port reads) straight on the ATTiny84,
     Here is the table from the library...

     PinChangeInterrupt Table

     Pins with * are not broken out/deactivated by default. You may activate them in the setting file (advanced).
     Each row section represents a port(0-3). Not all MCUs have all Ports/Pins physically available.

    | PCINT |     Attiny 84   |   Encoder Pins   |
    | ----- |  -------------- | ---------------- |
    |     0 |  0       (PA0)  |                  |
    |     1 |  1       (PA1)  |                  |
    |     2 |  2       (PA2)  |  Encoder 1 B     |
    |     3 |  3       (PA3)  |                  |
    |     4 |  4 SCK   (PA4)  |                  |
    |     5 |  5 MISO  (PA5)  |                  |
    |     6 |  6 MOSI  (PA6)  |                  |
    |     7 |  7       (PA7)  |  Encoder 2 A     |
    | ----- | ---------------
    |     8 | 10 XTAL1 (PB0)* |  Encoder 1 A     |
    |     9 |  9 XTAL2 (PB1)* |                  |
    |    10 |  8 INT0  (PB2)* |  Encoder 2 B     |
    |    11 |  RST     (PB3)* |                  |
  */

  // read in full port values for port A and port B on the ATTINY
  byte fullPortA = PINA;
  byte fullPortB = PINB;

  // pull out MSB1 and LSM from full port on both encoders.
  if (fullPortB & B00000001) MSB1 = 1;  // Encoder 1 A (D10) (PB0)
  if (fullPortA & B00000100) LSB1 = 1;  // Encoder 1 B (D2)  (PA2)

#endif


  //A complete indent occurs across four interrupts and looks like:
  //ABAB.ABAB = 01001011 for CW
  //ABAB.ABAB = 10110100
  //ABAB.ABAB = 00101101
  //ABAB.ABAB = 11010010
  
  //ABAB.ABAB = 10000111 for CCW
  //ABAB.ABAB = 01111000
  //ABAB.ABAB = 00011110
  //ABAB.ABAB = 11100001

  //lastEncoded1 could be many things but by looking for two unique values
  //we filter out corrupted and partially dropped encoder readings
  //Gaurantees we will not get partial indent readings

  byte encoded1 = (MSB1 << 1) | LSB1; //Convert the 2 pin value to single number
  lastEncoded1 = (lastEncoded1 << 2) | encoded1; //Add this to the previous readings

  // Encoder 1
  if ((lastEncoded1 == 0b01001011) || // clockwise
  (lastEncoded1 == 0b10110100) || 
  (lastEncoded1 == 0b00101101) || 
  (lastEncoded1 == 0b11010010)) 
  {
    registerMap.encoder1Count++;
        //If rotationLimit feature turned on, don't let the encoder1Count go past this value
        if (registerMap.rotationLimit)
        {
          if (registerMap.encoder1Count >= (int16_t)registerMap.rotationLimit)
          {
            registerMap.encoder1Count = 0;
          }
        }
        registerMap.encoder1Difference++;
    
    
        //We have moved one full tick so update moved bit and timestamp
        registerMap.status |= (1 << statusEncoderMovedBit); //Set the status bit to true to indicate movement
        lastEncoderTwistTime = millis(); //Timestamp this event
  }
  else if ((lastEncoded1 == 0b10000111) || // counter clockwise
  (lastEncoded1 == 0b01111000) ||
  (lastEncoded1 == 0b00011110) ||
  (lastEncoded1 == 0b11100001)) 
  {
    registerMap.encoder1Count--;
        // If rotationLimit feature turned on, don't let the encoder1Count go below zero
        if (registerMap.rotationLimit)
        {
          if (registerMap.encoder1Count < 0)
          {
            registerMap.encoder1Count = registerMap.rotationLimit;
          }
        }
        registerMap.encoder1Difference--;
    
        //We have moved one full tick so update moved bit and timestamp
        registerMap.status |= (1 << statusEncoderMovedBit); //Set the status bit to true to indicate movement
        lastEncoderTwistTime = millis(); //Timestamp this event
  }

}

void updateEncoder2() {

  byte MSB2 = 0;
  byte LSB2 = 0;

#if defined(__AVR_ATmega328P__)
  byte fullPortReadValue = PIND;// Read entire port D, encoders are on D2, D3, D4 and D5, so bits: 00?? ??00 (aka PD2, PD3, PD4, PD5)

  // pull out MSB and LSM from full port on both encoders.
  // encoder2 is on 4 and 5, so B00??0000
  if (fullPortReadValue & B00010000) MSB2 = 1;
  if (fullPortReadValue & B00100000) LSB2 = 1;

#elif defined(__AVR_ATtiny84__)

  // read in full port values for port A and port B on the ATTINY
  byte fullPortA = PINA;
  byte fullPortB = PINB;

  // pull out MSB and LSM from full port on both encoders.
  if (fullPortA & B10000000) MSB2 = 1;  // Encoder 2 A (D7)  (PA7)
  if (fullPortB & B00000100) LSB2 = 1;  // Encoder 2 B (D8)  (PB2)

#endif

  // Encoder 2

  byte encoded2 = (MSB2 << 1) | LSB2; //Convert the 2 pin value to single number
  lastEncoded2 = (lastEncoded2 << 2) | encoded2; //Add this to the previous readings


  if ((lastEncoded2 == 0b01001011) || // clockwise
  (lastEncoded2 == 0b10110100) || 
  (lastEncoded2 == 0b00101101) || 
  (lastEncoded2 == 0b11010010)) 
  {
    registerMap.encoder2Count++;
        // If rotationLimit feature turned on, don't let the encoder2Count go past this value
        if (registerMap.rotationLimit)
        {
          if (registerMap.encoder2Count >= (int16_t)registerMap.rotationLimit)
          {
            registerMap.encoder2Count = 0;
          }
        }
        registerMap.encoder2Difference++;
    
    
        //We have moved one full tick so update moved bit and timestamp
        registerMap.status |= (1 << statusEncoderMovedBit); //Set the status bit to true to indicate movement
        lastEncoderTwistTime = millis(); //Timestamp this event
  }
  else if ((lastEncoded2 == 0b10000111) || // counter clockwise
  (lastEncoded2 == 0b01111000) ||
  (lastEncoded2 == 0b00011110) ||
  (lastEncoded2 == 0b11100001)) 
  {
    registerMap.encoder2Count--;
        // If rotationLimit feature turned on, don't let the encoder2Count go below zero
        if (registerMap.rotationLimit)
        {
          if (registerMap.encoder2Count < 0)
          {
            registerMap.encoder2Count = registerMap.rotationLimit;
          }
        }
        registerMap.encoder2Difference--;
    
        //We have moved one full tick so update moved bit and timestamp
        registerMap.status |= (1 << statusEncoderMovedBit); //Set the status bit to true to indicate movement
        lastEncoderTwistTime = millis(); //Timestamp this event
  }

}
//Turn on interrupts for the various pins
void setupInterrupts()
{
  //Call updateEncoder() when any high/low changed seen on interrupt 0 (pin 2), or interrupt 1 (pin 3)
#if defined(__AVR_ATmega328P__)
  attachInterrupt(0, updateEncoder1, CHANGE);
  attachInterrupt(1, updateEncoder1, CHANGE);
#endif

#if defined(__AVR_ATtiny84__)
  //To make this line work you have to comment out https://github.com/NicoHood/PinChangeInterrupt/blob/master/src/PinChangeInterruptSettings.h#L228
  attachPCINT(digitalPinToPCINT(encoder1APin), updateEncoder1, CHANGE); //Doesn't work

  attachPCINT(digitalPinToPCINT(encoder1BPin), updateEncoder1, CHANGE);
#endif

  //Attach interrupt to encoder 2 pins -- needs pinchangeinterrupt library style interrupts
  attachPCINT(digitalPinToPCINT(encoder2APin), updateEncoder2, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2BPin), updateEncoder2, CHANGE);
}

//When Twist receives data bytes from Master, this function is called as an interrupt
void receiveEvent(int numberOfBytesReceived)
{
  registerNumber = Wire.read(); //Get the memory map offset from the user

  //Begin recording the following incoming bytes to the temp memory map
  //starting at the registerNumber (the first byte received)
  for (byte x = 0 ; x < numberOfBytesReceived - 1 ; x++)
  {
    byte temp = Wire.read(); //We might record it, we might throw it away

    if ( (x + registerNumber) < sizeof(memoryMap))
    {
      //Clense the incoming byte against the read only protected bits
      //Store the result into the register map
      *(registerPointer + registerNumber + x) &= ~*(protectionPointer + registerNumber + x); //Clear this register if needed
      *(registerPointer + registerNumber + x) |= temp & *(protectionPointer + registerNumber + x); //Or in the user's request (clensed against protection bits)
    }
  }

  if (interruptState == STATE_INT_INDICATED)
  {
    //If the user has cleared all the interrupt bits then clear interrupt pin
    if ( (registerMap.status & (1 << statusEncoderMovedBit)) == 0)
    {
      //This will set the int pin to high impedance (aka pulled high by external resistor)
      digitalWrite(interruptPin, LOW); //Push pin to disable internal pull-ups
      pinMode(interruptPin, INPUT); //Go to high impedance

      interruptState = STATE_INT_CLEARED; //Go to next state
    }
  }

  updateOutputs = true; //Update things like LED brightnesses in the main loop
}

//Respond to GET commands
//When Twist gets a request for data from the user, this function is called as an interrupt
//The interrupt will respond with bytes starting from the last byte the user sent to us
//While we are sending bytes we may have to do some calculations
void requestEvent()
{
  //Calculate time stamps before we start sending bytes via I2C
  if (lastEncoderTwistTime > 0) registerMap.timeSinceLastMovement = millis() - lastEncoderTwistTime;

  //This will write the entire contents of the register map struct starting from
  //the register the user requested, and when it reaches the end the master
  //will read 0xFFs.
  Wire.write((registerPointer + registerNumber), sizeof(memoryMap) - registerNumber);
}
