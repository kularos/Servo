/*
 * --- Servo interface ---
 * In general, a servo is comprised of a set of inputs, outputs and a control function.
 * To simplify, these c++ sensors will be 100% transparent (no feedback or feed-forward)
 *    - Thus, they don't need to worry about control loops or error.
 *  
 *  The only methods they need to implement in order to fit the interface are:
 *    - open a stream:      OPEN()
 *    - close a stream:     ClOSE()
 *    - set control vector: SET_CTL() 
 *    - get sensor data:    GET_SNS()
 *  
 *  As these are physical devices, they also stand to implement methods to specify the physical parameters of the device:
 *    - retrieve information about the device: GET_INFO() is going to be an asynchronous communication (duration unspecified) and should include:
 *      (read only) - An identifier that is unique to the device.
 *      (read only) - Specifications on the expected number of bytes in each communication (GET_SNS, SET_CTL, CALIBRATE), and how these bytes correspnd to an i/o vector.
 *      - These two communications should be separated to allow the upladder node to 
 *    
 */

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

 Adafruit_SHT31 sht3x = Adafruit_SHT31();

// Script parameters:
#define DEBUG false
#define ID 0x4944


// Serial interface byte definitions:
#define END_BYTE       0x0A

#define OPEN           0x20
#define GET_ID         0x21
#define CLOSE          0x22
#define RESET          0x23
#define GET_SENSE      0x24
#define SET_CONTROL    0x25

#define NO_ERROR       0x40  // No error occured during command execution.
#define CMD_VAL_ERROR  0x41  // An unknown command byte was provided.
#define CMD_NUL_ERROR  0x42  // A command was executed before assignment.
#define CMD_EXE_ERROR  0x43  // An error occured during command execution.
#define DAT_IDX_ERROR  0x44  // The provided amount of data was not consistent with the expected amount for the command.
#define RSP_IDX_ERROR  0x45  // The generated response was not consistent with the expected length for the command.
#define SENSOR_ERROR   0x49
#define CONTROL_ERROR  0x4A


// Initialize Serial flags, counters, and buffers.

uint8_t inputIndex = 0;
byte inputBuffer  [8];

uint8_t outputIndex = 0;
byte outputBuffer [8];

int16_t cmdIndex = -1; //-1 for unassigned, -2 for unknown.
uint8_t bytesToRead = 0;
bool cmdInProgress = false;


// Interface to update servo vectors.
/* These are hardware-dependent, and should return a boolean of whether or not the update was successful.*/

const uint8_t nCnt = 2;
uint16_t cntVec [nCnt];
const uint8_t cntPins [nCnt] = {6, 9};

const uint8_t nSns = 2;
const uint8_t bytesPerSns = 2;
uint16_t snsVec [nSns];


bool updateControl(){
  // Iteratively form control vector by reading bytes from the input buffer.
  // This can be extended for control values that exceed 1 byte. 
  
  for(uint8_t i = 0; i < nCnt; i++){
    //Reset current value in Control vector:
    cntVec[i] = inputBuffer[i];
  }

  return true;
}


bool updateSense(){
  //Read climate data from sht31 library, normalize to 16 bits, and assign to sense vector.
  
  // Read Climate Data:
  float t = sht3x.readTemperature();
  float rh = sht3x.readHumidity();

  // Adafruit sht31 library marks a failed sesing measurement by setting the value to NaN. 
  if (! (isnan(t) || isnan(rh))){

    // Cast range of values to a single byte:
    uint16_t t_byte = (uint16_t)((t + 45.0f) * 13107.0f / 35.0f);
    uint16_t rh_byte = (uint16_t)(rh * 655.35f);
  
    //Update sense vector and return True (no error):
    snsVec[0] = t_byte; snsVec[1] = rh_byte;
    return true;
    
  } else {
    
    // Set sense vector values to 0 and return False (error in measurement):
    snsVec[0] = 0x00; snsVec[1] = 0x00;
    return false; 
  }
}


// --- Command Function Definitions ---

byte openDevice(){
  //Initialize parameters to reset state, and send identifier code over serial.
  //Initialize device in ground state.
  byte errorCode = resetDevice();
  return errorCode;
}


byte getIdentifier(){
  /*Set device's 2-byte identifier to the write buffer.*/
  
  //Write device ID to first 2 bytes of output buffer.
  outputBuffer[0] = ID >> 8;
  outputBuffer[1] = ID & 255;

  //Update output buffer indexer:
  outputIndex = 2;
  
  return NO_ERROR;
}
  

byte closeDevice(){
  /* Set internal parameters to known state and close the device.*/
  // Reset the device to its ground state before closing:
  byte errorCode = resetDevice();
  return errorCode;
}

 
byte resetDevice(){
  /*Places the device into a well defined state*/

  //Reset values of control vector and set pins to 0:
  for(uint8_t i = 0; i < nCnt; i++){cntVec[i] = 0; analogWrite(cntPins[i], 0);}

  //Reset sense vector and soft reset sht3x:
  for(uint8_t i = 0; i < nSns; i++){snsVec[i] = 0;}
  sht3x.reset();

  return NO_ERROR;
}


byte getSense(){
  //Update sense vector and send over serial.

  // If updateSense fails, send Sensor error.
  if(! updateSense()){return SENSOR_ERROR;}

  else{
    //Send each value of sense vector over serial.
    for(uint8_t i = 0; i < nSns; i++){
      
      uint16_t sense = snsVec[i];
      if(DEBUG == true){Serial.print(sense); Serial.print("\t");}
      
      //Write bytes following little endian since its easier:
      for(uint8_t j = 0; j < bytesPerSns; j++){
        
        //Write bottom byte to output buffer:
        outputBuffer[outputIndex] = sense & 255;
  
        //Shift out bottom byte of sense:
        sense >>= 8;

         //Increment output buffer index:
         outputIndex ++;
      }
    }

    if(DEBUG == true){Serial.print("\n");}
    return NO_ERROR;
  }
} 


byte setControl(){
  //Iterate through all control pins and assign the corresponding value of the control vector.
  if(! updateControl()){return CONTROL_ERROR;}
  
  else{
    //Write control vector values to pins:
    for(uint8_t i = 0; i < nCnt; i++){
      
      analogWrite(cntPins[i], cntVec[i]);
      
      if(DEBUG == true){Serial.print(cntVec[i]); Serial.print("\t");}
    }
    
    if(DEBUG == true){Serial.print("\n");}
    return NO_ERROR;
  }
}



// Sensor structure and definitions.
typedef struct{
} sensor;


// Actuator structure and definitions
typedef struct{
} actuator;


// Command structure and definitions.
typedef struct{
  byte byteCode;         // The command byte that triggers execution of this command.
  byte (*execute)(void); // A pointer to the function that is called upon this command's execution.
  uint8_t dataBytes;     // The number of data bytes that are expected to be transmitted alongside this command.
  uint8_t responseBytes; // The number of bytes that are expected to comprise this command's response.
} command;

const uint8_t nCmd = 6;
command commands [nCmd] = {
  {OPEN,        &openDevice    , 0, 0},
  {GET_ID,      &getIdentifier , 0, 2},
  {CLOSE,       &closeDevice   , 0, 0},
  {RESET,       &resetDevice   , 0, 0},
  {GET_SENSE,   &getSense      , 0, 4},
  {SET_CONTROL, &setControl    , 2, 0}
  };


// --- Serial Interface ---:

void parseCommand(byte byteCode){
  /*Takes in a command byte, and sets commandFunction to point to the corresponding function*/
  
  // Defaults to the flag of an unknown command, and no databytes to read
  cmdIndex = -2;
  bytesToRead = 0;

  // Compare byteCode to those of all commands.
  for(uint8_t i = 0; i < nCmd; i++){
    
    command cmd = commands[i];
    if(cmd.byteCode == byteCode){

      // Set command address and number of databytes to read.
      cmdIndex = i;
      bytesToRead = cmd.dataBytes; 
      break;
      }
  }
}

byte executeCommand(){
  /* Ensure that the command is well formed, attempt to execute it, and return the appropriate error code.*/
  
  // Manage errors corresponding to malformed commands:
  if(cmdIndex == -1){return CMD_NUL_ERROR;}
  if(cmdIndex == -2){return CMD_VAL_ERROR;}

  // Ensure that the provided data is consistent with the desired command:
  command cmd = commands[cmdIndex];
  // if(cmd.dataBytes != inputIndex){return DAT_IDX_ERROR;} // No longer needed, as we only read the number of data bytes required.

  // Execute command and ensure response has expected length (command execution errors take priority over response length errors):
  // Any response generated by cmd.func will still be sent.
  byte executionError = cmd.execute();
  if(executionError != NO_ERROR){return executionError;}
  if(cmd.responseBytes != outputIndex){return RSP_IDX_ERROR;}

  // Otherwise:
  return NO_ERROR;
}


void receiveCommand(){
  
  if(Serial.available() > 0){
    byte newByte = Serial.read();

    // If a command is not in progress, interpret newByte as a command.
    if(! cmdInProgress){
      parseCommand(newByte);
      cmdInProgress = true;
      
      if(DEBUG == true){Serial.print(F("Command started.\n"));}
    }

    // If a command is in progress, interpret newByte as data: 
    else{
      inputBuffer[inputIndex] = newByte;
      
      // Increment counters
      bytesToRead --;
      inputIndex ++;

      if(DEBUG == true){Serial.print(F("Recieved a byte of data.\n"));}      
    }

    // If the command is now over, execute it:
    if(bytesToRead == 0 && cmdInProgress){

      // Execute command and write output buffer over serial.
      byte errorCode = executeCommand();
      sendResponse(errorCode);

      // Reset flags:
      cmdInProgress = false;
      cmdIndex = -1;

      outputIndex = 0;
      inputIndex = 0;
      
      if(DEBUG == true){Serial.println(); Serial.print(F("Command Executed.\n\n"));}
    }  
  }
  
}


void sendResponse(byte errorCode){
  /* Execute the pointed command function, write any response over serial, and reset.*/
  
  // Call to command function is handled in validateCommand. Write the error code over serial:
  Serial.write(errorCode);

  // Write output buffer over serial.
  for(uint8_t i = 0; i < outputIndex; i++){Serial.write(outputBuffer[i]);}
}


// --ARDUINO RUNTIME--
void setup(){
  
  // Open serial interface and block until console is opened.
  Serial.begin(9600);
  while(!Serial){delay(10);}
  if(DEBUG == true){Serial.print(F("Serial Opened.\n\n"));}

  // Start I2C interface with sht3x at address 0x44.
  if(! sht3x.begin(0x44)){ 
    if(DEBUG == true){Serial.print(F("Couldn't find sht3x at provided address.\n\n"));}
    while (1) delay(1);
  }
  
  // Set output pins to output
  for(uint8_t i = 0; i < nCnt; i++){pinMode(cntPins[i], OUTPUT);}
}


void loop(){ 
  receiveCommand();
}
