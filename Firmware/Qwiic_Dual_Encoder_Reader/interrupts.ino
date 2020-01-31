//Called every time the encoder is twisted
//Based on http://bildr.org/2012/08/rotary-encoder-arduino/
//Adam - I owe you many beers for bildr
void updateEncoder() {

  byte MSB1 = 0;
  byte LSB1 = 0;
  byte MSB2 = 0;
  byte LSB2 = 0;  

#if defined(__AVR_ATmega328P__)
  byte fullPortReadValue = PIND;// Read entire port D, encoders are on D2, D3, D4 and D5, so bits: 00?? ??00 (aka PD2, PD3, PD4, PD5)

  // pull out MSB1 and LSM from full port on both encoders.
  // encoder1 is on 2 and 3, so B0000??00
  if (fullPortReadValue & B00000100) MSB1 = 1;
  if (fullPortReadValue & B00001000) LSB1 = 1;
  // encoder2 is on 4 and 5, so B00??0000
  if (fullPortReadValue & B00010000) MSB2 = 1;
  if (fullPortReadValue & B00100000) LSB2 = 1;

#elif defined(__AVR_ATtiny84__)
  byte fullPortReadValue = PINA; // A

  // pull out MSB1 and LSM from full port on both encoders.

#endif

  //  MSB1 = digitalRead(encoder1APin); //MSB1 = most significant bit
  //  LSB1 = digitalRead(encoder1BPin); //LSB1 = least significant bit

  byte encoded1 = (MSB1 << 1) | LSB1; //Convert the 2 pin value to single number
  lastEncoded1 = (lastEncoded1 << 2) | encoded1; //Add this to the previous readings

  byte encoded2 = (MSB2 << 1) | LSB2; //Convert the 2 pin value to single number
  lastEncoded2 = (lastEncoded2 << 2) | encoded2; //Add this to the previous readings  

  //A complete indent occurs across four interrupts and looks like:
  //ABAB.ABAB = 01001011 for CW
  //ABAB.ABAB = 10000111 for CCW

  //lastEncoded1 could be many things but by looking for two unique values
  //we filter out corrupted and partially dropped encoder readings
  //Gaurantees we will not get partial indent readings

  if (lastEncoded1 == 0b01001011) //One indent clockwise
  {
    registerMap.encoder1Count++;
    // If rotationLimit feature turned on, don't let the encoder1Count go past this value
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
  else if (lastEncoded1 == 0b10000111) //One indent counter clockwise
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

  if (lastEncoded2 == 0b01001011) //One indent clockwise
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
  else if (lastEncoded2 == 0b10000111) //One indent counter clockwise
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
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
#endif

#if defined(__AVR_ATtiny84__)
  //To make this line work you have to comment out https://github.com/NicoHood/PinChangeInterrupt/blob/master/src/PinChangeInterruptSettings.h#L228
  attachPCINT(digitalPinToPCINT(encoder1APin), updateEncoder, CHANGE); //Doesn't work

  attachPCINT(digitalPinToPCINT(encoder1BPin), updateEncoder, CHANGE);
#endif

  //Attach interrupt to encoder 2 pins -- needs pinchangeinterrupt library style interrupts
  attachPCINT(digitalPinToPCINT(encoder2APin), updateEncoder, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder2BPin), updateEncoder, CHANGE);
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
