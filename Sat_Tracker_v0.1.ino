#include <math.h>
#include <LiquidCrystal.h>
#include <Wire.h>             //change declercation in LSM303d library also
#include <LSM303.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// Very messy but still functional alpha version by KN4AKM //

//#define DEBUG

//                                                                          //
//                              Rotor config                                //
//                                                                          //

//Disable analog overlap for Mike's configuration
#define analogOverlapSense        //Enable overlap sensing via linear potentiometer in base of AZ rotor
#define analogOverlapPin A0       //The pin our overlap potentiometer's wiper is on


#define rotorRunPin 6
#define rotorRunActive LOW        //Run relay active state

#define rotorDirectionPin 7
#define rotorDirectionActive LOW     //Direction relay active state (CCW)

#define rotorSelectPin 8
#define rotorSelectActive LOW     //Rotor select relay active state (EL)

int _homeAzimuth = 20;              //AZ Home position
int _homeElevation = 1;             //EL Home position

const int _maxRotorAzimuth = 360;  // maximum rotor azimuth in degrees
const int _maxRotorElevation = 90; // maximum rotor elevation in degrees

const int _overlapToleranceLow = 10; //highest number on the low end of AZ that will register as an overlap, if newAZ < #
const int _overlapToleranceHigh = 350; //highest number on the low end of AZ that will register as an overlap, if newAZ > #

//const int _azelAveraging = 6;      // number of times to sample az/el per reading
//const int _azelAveragingDelay = 1; // ms to delay between measurment

int _azcloseEnough = 2;
int _elcloseEnough = 4;
int _rotorAzimuth = 0;       // current rotor azimuth in degrees

int _analogOverlapCount = 0;
int _analogRotorAzimuth = 0;

int _rotorElevation = 0;     // current rotor azimuth in degrees
int _azimuthTemp = 0;        // used for gs232 azimuth decoding
int _elevationTemp = 0;      // used for gs232 elevation decoding  
int _newAzimuth = 0;         // new azimuth for rotor move
int _newElevation = 0;       // new elevation for rotor move
int _previousRotorAzimuth = 0;       // previous rotor azimuth in degrees
int _previousRotorElevation = 0;     // previous rotor azimuth in degrees
int _azOverlapCount = 0;
boolean checkOverlap = false;

unsigned long _rtcLastCalibrationStore = 0UL;         
unsigned long _CalibrationStoreInterval = 30000UL; //interval to check for updated cal values and store in EEPROM
unsigned long _rtcLastAZELUpdate = 0UL;         
unsigned long _rtcLastDisplayUpdate = 0UL;      // rtc at start of last loop
unsigned long _rtcLastRotorUpdate = 0UL;        // rtc at start of last loop
unsigned long _displayUpdateInterval = 500UL;   // display update interval in mS
unsigned long _rotorMoveUpdateInterval = 100UL; // rotor move check interval in mS  - was 100
unsigned long _AZELUpdateInterval = 50UL; // AZ/EL update interval in mS


boolean _gs232WActive = false;  // gs232 W command in process
int _gs232AzElIndex = 0;        // position in gs232 Az El sequence
long _gs232Azimuth = 0;          // gs232 Azimuth value
long _gs232Elevation = 0;        // gs232 Elevation value
boolean _azimuthMove = false;     // azimuth move needed
boolean _elevationMove = false;   // elevation move needed

boolean elIsMoving = false;
boolean azIsMoving = false;



String azRotorMovement = "-";   // string for az rotor move display
String elRotorMovement = "-";  // string for el rotor move display

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
LSM303 compass;

//LSM303::vector<int16_t> selfcal_min = {32767, 32767, 32767}, selfcal_max = {-32768, -32768, -32768};


bool storeMinX = false;
bool storeMinY = false;
bool storeMinZ = false;
bool storeMaxX = false;
bool storeMaxY = false;
bool storeMaxZ = false;

//                                                                          //
//                        Run once at startup/reset                         //
//                                                                          //

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  //Wire.setClock(31000L);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  
  compass.init();
  compass.enableDefault();                                                  // Default uncalibrated reference:
  //compass.m_min = (LSM303::vector<int16_t>){ -2932,  -2922,  -2749};           // compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  //compass.m_max = (LSM303::vector<int16_t>){ +2907,  +1969,  +3293};           // compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  //{ -2932,  -2922,  -2749}    max: { +2907,  +1969,  +3293}



  #ifdef DEBUG 
    Serial.println("DEBUG mode started.");
  #endif
    
  pinMode(rotorRunPin, OUTPUT);
  pinMode(rotorDirectionPin, OUTPUT);
  pinMode(rotorSelectPin, OUTPUT);
  rotate_STOP();

                        //    Watchdog timer threshold interval - safest above 1 second
                        // 8Sec    4Sec    2Sec    1Sec     500mS      250mS      120mS
  wdt_enable(WDTO_2S);  //WDTO_8S WDTO_4S WDTO_2S WDTO_1S WDTO_500MS WDTO_250MS WDTO_120MS


  #ifndef analogOverlapSense
  int EEPROMtemp = EEPROMReadInt(0);            //Overlap count occupies EEPROM bytes 0 and 1
  if ((EEPROMtemp > -1) && (EEPROMtemp < 21)) {
    _azOverlapCount = EEPROMtemp-10;
  }
  #endif

  int EEPROMMin[3] = {-32767, -32767, -32767};
  int EEPROMMax[3] = {+32767, +32767, +32767};

  //Fetch compass calibration values from EEPROM at Nth byte+1
  EEPROMMin[0] = EEPROMReadInt(2);   
  EEPROMMin[1] = EEPROMReadInt(4);
  EEPROMMin[2] = EEPROMReadInt(6);
  
  EEPROMMax[0] = EEPROMReadInt(8);
  EEPROMMax[1] = EEPROMReadInt(10);
  EEPROMMax[2] = EEPROMReadInt(12);

  //Remove // from the following line to reset calibration, be sure to replace the // after resetting and reupload the program.
  //EEPROMMin[0] = -1;

  //check if eeprom values make sense.
  if ((EEPROMMin[0] != -1) && (EEPROMMin[1] != -1) && (EEPROMMin[2] != -1) && (EEPROMMax[0]  != -1) && (EEPROMMax[1]  != -10) && (EEPROMMax[2]  != -1)) {
    compass.m_min = (LSM303::vector<int16_t>){ EEPROMMin[0],  EEPROMMin[1],  EEPROMMin[2]};
    compass.m_max = (LSM303::vector<int16_t>){ EEPROMMax[0],  EEPROMMax[1],  EEPROMMax[2]};
    Serial.println("Read compass calibration values from EEPROM:");
    Serial.print("Min: { "); Serial.print(EEPROMMin[0]); Serial.print(", "); Serial.print(EEPROMMin[1]); Serial.print(", "); Serial.print(EEPROMMin[2]); Serial.println(" }");
    Serial.print("Max: { "); Serial.print(EEPROMMax[0]); Serial.print(", "); Serial.print(EEPROMMax[1]); Serial.print(", "); Serial.print(EEPROMMax[2]); Serial.println(" }");

    //selfcal_min = compass.m_min;
    //selfcal_max = compass.m_max;
  } else {
    //Begin auto calibration
    compass.m_min = (LSM303::vector<int16_t>){32767, 32767, 32767};
    compass.m_max = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    Serial.println("Starting auto calibration.");
    //selfcal_min = compass.m_min;
    //selfcal_max = compass.m_max;
  }

  //Remove // from the following line to reset overlap count, be sure to replace the // after resetting and reupload the program.
  //_azOverlapCount=0; EEPROMWriteInt(0,_azOverlapCount+10);

  readAZEL();
  _previousRotorAzimuth = _rotorAzimuth;
  _previousRotorElevation = _rotorElevation;    
}


//                                                                          //
//                             Main program loop                            //
//                                                                          //

void loop() {      
  // check for serial data
  if (Serial.available() > 0) {
    decodeGS232(Serial.read()); 
  }
    
  unsigned long rtcCurrent = millis(); // get current rtc value

  if (rtcCurrent > _rtcLastAZELUpdate+_AZELUpdateInterval) {
    readAZEL(); // get current azimuth/elevation

    _rtcLastAZELUpdate = rtcCurrent;
  }

  // check for calibration updates
  if (rtcCurrent > _rtcLastCalibrationStore+_CalibrationStoreInterval) {

    bool didStore = false;
    
    if (storeMinX) {
      EEPROMWriteInt(2,compass.m_min.x);
      storeMinX = false;
      didStore = true;
    }
    if (storeMinY) {
      EEPROMWriteInt(4,compass.m_min.y);
      storeMinY = false;
      didStore = true;
    }
    if (storeMinZ) {
      EEPROMWriteInt(6,compass.m_min.z);
      storeMinZ = false;
      didStore = true;
    }
    if (storeMaxX) {
      EEPROMWriteInt(8,compass.m_max.x);
      storeMaxX = false;
      didStore = true;
    }
    if (storeMaxY) {
      EEPROMWriteInt(10,compass.m_max.y);
      storeMaxY = false;
      didStore = true;
    }
    if (storeMaxZ) {
      EEPROMWriteInt(12,compass.m_max.z);
      storeMaxZ = false;
      didStore = true;
    }

    if (didStore) {
      Serial.println("Stored new calibration values to EEPROM:");
      Serial.print("Min: { "); Serial.print(compass.m_min.x); Serial.print(", "); Serial.print(compass.m_min.y); Serial.print(", "); Serial.print(compass.m_min.z); Serial.println(" }");
      Serial.print("Max: { "); Serial.print(compass.m_max.x); Serial.print(", "); Serial.print(compass.m_max.y); Serial.print(", "); Serial.print(compass.m_max.z); Serial.println(" }");
    }
  
    _rtcLastCalibrationStore = rtcCurrent;
  }
//  }
  
  // check for rtc overflow - skip this cycle if overflow
  if (rtcCurrent > _rtcLastDisplayUpdate) { // overflow if not true    _rotorMoveUpdateInterval
    // update rotor movement if necessary
    if (rtcCurrent - _rtcLastRotorUpdate > _rotorMoveUpdateInterval) {
      _rtcLastRotorUpdate = rtcCurrent; // reset rotor move timer base

      // AZIMUTH - check if azimuth move is required    
      if (_azimuthMove) {               
        if ( (abs(_rotorAzimuth - _newAzimuth) > _azcloseEnough) )  {
          updateAzimuthMove();
        } else { // no move required - turn off azimuth rotor
          rotate_STOP();
          _azimuthMove = false;
        }
      } else if (_elevationMove) {
         
        // ELEVATION - check if elevation move is required
        if ( abs(_rotorElevation - _newElevation) > _elcloseEnough) { //move required
          updateElevationMove();
        } else { // no move required - turn off elevation rotor
          rotate_STOP();
          _elevationMove = false;
        }
      } else {
        rotate_STOP();
        _azimuthMove = false;
        _elevationMove = false;
        azRotorMovement = "        ";
        elRotorMovement = "        ";
      }
    } // end of update rotor move
      
    // update display if necessary
    if (rtcCurrent - _rtcLastDisplayUpdate > _displayUpdateInterval) {

      // update rtcLast 
      _rtcLastDisplayUpdate = rtcCurrent;  // reset display update counter base
      displayAzEl(_rotorAzimuth, _rotorElevation);
    } 
  } else { // rtc overflow - just in case
    // update rtcLast 
    _rtcLastDisplayUpdate = rtcCurrent;
  } 
  wdt_reset();
}


//                                                                          //
//                       Update Elevation rotor move                        //
//                                                                          //

void updateElevationMove() {          
  // calculate rotor move 
  long rotorMoveEl = _newElevation - _rotorElevation;
   
  if (rotorMoveEl > 0) {
    elRotorMovement = "U";
    elRotorMovement = elRotorMovement + String(_newElevation);
    rotateEL_UP();   
  } else {           
    if (rotorMoveEl < 0) {
      elRotorMovement = "D";
      elRotorMovement = elRotorMovement + String(_newElevation);
      rotateEL_DOWN();   
    } 
  } 
}


//                                                                          //
//                        Update Azimuth rotor move                         //
//                                                                          //

void updateAzimuthMove() {          
  // calculate rotor move 

  long rotorMoveAz;

  if ((_newAzimuth == _homeAzimuth) && (_newElevation == _homeElevation)) {
    if (_azOverlapCount > 0) {
      rotorMoveAz = _newAzimuth - _rotorAzimuth - 360;
    } else if (_azOverlapCount < 0) {
      rotorMoveAz = _newAzimuth - _rotorAzimuth + 360;
    }

    #ifdef DEBUG
      Serial.print("Return Overlaps: ");
      Serial.print(_azOverlapCount);
      Serial.print(" rotorMoveAz: ");
      Serial.println(rotorMoveAz);
    #endif
    
    exit;
  } else {

    rotorMoveAz = _newAzimuth - _rotorAzimuth;
    // adjust move if necessary
    #ifdef DEBUG
      Serial.print("updateAzimuthMove([");
    #endif
    if (rotorMoveAz > 180) { // adjust move if > 180 degrees
      rotorMoveAz = rotorMoveAz - 360;

      #ifdef DEBUG
        Serial.print("-");
      #endif
    } else {           
      if (rotorMoveAz < -180) { // adjust move if < -180 degrees 
        rotorMoveAz = rotorMoveAz + 360;
      
        #ifdef DEBUG
          Serial.print("+");
        #endif
      }
    }
  
    #ifdef DEBUG
      Serial.print("]): _newAzimuth=");
      Serial.print(_newAzimuth);
      Serial.print(", _rotorAzimuth=");
      Serial.print(_rotorAzimuth);
      Serial.print(", rotorMoveAz=");
      Serial.println(rotorMoveAz);
    #endif
  
  }


  if (rotorMoveAz > 0) {
    azRotorMovement = "R";
    azRotorMovement = azRotorMovement + String(_newAzimuth);
    rotateAZ_CW();
  } else {   
    if (rotorMoveAz < 0) {
      azRotorMovement = "L";
      azRotorMovement = azRotorMovement + String(_newAzimuth);
      rotateAZ_CCW();
    }
  }
}


//                                                                          //
//                 Read azimuth and elevation from LSM303d                  //
//                                                                          //

double analogOverlapMap[11] = {0,102.4,204.8,307.2,409.6,512,614.4,716.8,819.2,921.6,1024};

void readAZEL() {

  compass.read();
  _rotorAzimuth = compass.heading();

  //Check if calibration should be updated
  if (compass.m.x < compass.m_min.x) {
    compass.m_min.x = compass.m.x;
    //Serial.print("minX: " );
    //Serial.println(compass.m_min.x);
    storeMinX = true;
  }
  if (compass.m.y < compass.m_min.y) {
    compass.m_min.y = compass.m.y;
    storeMinY = true;
  }
  if (compass.m.z < compass.m_min.z) {
    compass.m_min.z = compass.m.z;
    storeMinZ = true;
  }

  //Check if calibration should be updated
  if (compass.m.x > compass.m_max.x) {
    compass.m_max.x = compass.m.x;
    //Serial.print("maxX: " );
    //Serial.println(compass.m_max.x);
    storeMaxX = true;
  }
  if (compass.m.y > compass.m_max.y) {
    compass.m_max.y = compass.m.y;
    storeMaxY = true;
  }
  if (compass.m.z > compass.m_max.z) {
    compass.m_max.z = compass.m.z;
    storeMaxZ = true;
  }


  
  _rotorElevation = ((atan2(compass.a.x, compass.a.z) * -180) / M_PI)*-1;


  #ifndef analogOverlapSense  //Digital overlap sense

    //Check for overlaps
    if ((_previousRotorAzimuth < 10) && (_rotorAzimuth > 350)) {
      _azOverlapCount--;
      EEPROMWriteInt(0,_azOverlapCount+10);
    } else if ((_previousRotorAzimuth > 350) && (_rotorAzimuth < 10)) {
      _azOverlapCount++;        
      EEPROMWriteInt(0,_azOverlapCount+10);
    }

  #else  //Analog overlap sense

    int analogReadVal = analogRead(analogOverlapPin);
    int analogTestVal = 0;

    for (int i=0; i < 11; i++) {
      analogTestVal = analogOverlapMap[i]-analogReadVal;
      if (abs(analogTestVal) < 52) {
        _azOverlapCount = 5-i;
        i=11;
      }
    }
  
  #endif
  
  wdt_reset();
}

//                                                                          //
//                   Print Azimuth & Elevation on display                   //
//                                                                          //

void displayAzEl(long az, long el) { 
  _previousRotorAzimuth = _rotorAzimuth;
  _previousRotorElevation = _rotorElevation;

  String cardAZ = convertDegreeToCardinalDirection(az);

  lcd.setCursor(0,0);
  lcd.print(az);
  lcd.print((char)223);
  //lcd.print("/");
  //lcd.print(_analogRotorAzimuth);
  //lcd.print((char)223);
  lcd.print("          ");
  lcd.setCursor(15-azRotorMovement.length(),0);
  lcd.print(azRotorMovement);
  lcd.print((char)223);
  lcd.setCursor(7,0);
  lcd.print(_azOverlapCount);
  //lcd.print("  ");    

  lcd.setCursor(0,1);
  lcd.print(el);
  lcd.print((char)223);
  lcd.print("          ");
  lcd.setCursor(15-elRotorMovement.length(),1);
  lcd.print(elRotorMovement);
  lcd.print((char)223);

  if (cardAZ.length() == 3) {
  lcd.setCursor(6,1);
  } else {
    lcd.setCursor(7,1);
  }
  
  lcd.print(cardAZ);

  //lcd.print("  ");   

}


//                                                                          //
//                        Print stats to serial port                        //
//                                                                          //

void printStats()
{
  Serial.print("A");
  Serial.print(_rotorAzimuth);
  Serial.print("E");
  Serial.print(_rotorElevation);
  Serial.print("O");
  Serial.print(_azOverlapCount);
  
  Serial.println();
}


//                                                                          //
//                          Rotor control routines                          //
//                                                                          //

// Stop all rotation
void rotate_STOP() {                     //Stop rotation
  azIsMoving = false;
  elIsMoving = false;
  digitalWrite(rotorRunPin,!rotorRunActive);        //Remove power supply to rotors
  digitalWrite(rotorSelectPin,!rotorSelectActive);     //Rotor select to default value
  digitalWrite(rotorDirectionPin,!rotorDirectionActive);  //Rotor direction to default value
  #ifdef DEBUG 
    Serial.println("rotate_STOP()");
  #endif
}

// Rotate azimuth clockwise
void rotateAZ_CW() {                     //Rotate azimuth clockwise-AZ angle increase
  azIsMoving = true;
  digitalWrite(rotorSelectPin, !rotorSelectActive);     //Select azimuth rotor
  digitalWrite(rotorDirectionPin, !rotorDirectionActive);  //Select clockwise direction
  digitalWrite(rotorRunPin, rotorRunActive);         //Supply power to rotor
  
  #ifdef DEBUG 
    Serial.print("rotateAZ_CW(): _newAzimuth=");
    Serial.print(_newAzimuth);
    Serial.print(", _rotorAzimuth=");
    Serial.println(_rotorAzimuth);
  #endif
}

// Rotate azimuth counter-clockwise
void rotateAZ_CCW() {                    //Rotate azimuth counterclockwise-AZ angle decrease
  azIsMoving = true;
  digitalWrite(rotorSelectPin,!rotorSelectActive);     //Select azimuth rotor
  digitalWrite(rotorDirectionPin,rotorDirectionActive);   //Select counter-clockwise direction
  digitalWrite(rotorRunPin,rotorRunActive);         //Supply power to rotor

  #ifdef DEBUG 
    Serial.print("rotateAZ_CCW(): _newAzimuth=");
    Serial.print(_newAzimuth);
    Serial.print(", _rotorAzimuth=");
    Serial.println(_rotorAzimuth);
  #endif
}

// Rotate elevation up
void rotateEL_UP() {                     //Rotate Elevation clockwise-EL angle increase
  elIsMoving = true;
  digitalWrite(rotorSelectPin,rotorSelectActive);      //Select elevation rotor
  digitalWrite(rotorDirectionPin,!rotorDirectionActive);  //Select clockwise direction
  digitalWrite(rotorRunPin,rotorRunActive);         //Supply power to rotor
  
  #ifdef DEBUG 
    Serial.print("rotateEL_UP(): _newElevation=");
    Serial.print(_newElevation);
    Serial.print(", _rotorElevation=");
    Serial.println(_rotorElevation);
  #endif
}

// Rotate elevation down
void rotateEL_DOWN() {                   //Rotate Elevation counterclockwise-EL angle decrease
  elIsMoving = true;
  digitalWrite(rotorSelectPin,rotorSelectActive);      //Select elevation rotor
  digitalWrite(rotorDirectionPin,rotorDirectionActive);   //Select counter-clockwise direction
  digitalWrite(rotorRunPin,rotorRunActive);         //Supply power to rotor

  #ifdef DEBUG 
    Serial.print("rotateEL_DOWN(): _newElevation=");
    Serial.print(_newElevation);
    Serial.print(", _rotorElevation=");
    Serial.println(_rotorElevation);
  #endif
}


//                                                                          //
//                          Decode gs232 commands                           //
//                                                                          //

void decodeGS232(char character) {
  switch (character) {

    //Commands for custom software control

    case 'i':
    case 'I':{
      printStats();
    }

    case 'w':  // gs232 W command
    case 'W': {
      { _gs232WActive = true; _gs232AzElIndex = 0; }
        break;
      }
       
    // numeric - azimuth and elevation digits
    case '0':  case '1':   case '2':  case '3':  case '4':  case '5':  case '6':   case '7':  case '8':  case '9': {
      if (_gs232WActive) {
        processAzElNumeric(character);          
      }
    } 

    default: {
      // ignore everything else
    }
  }
}


//                                                                          //
//   Process Azimuth & Elevation numeric characters from gs232 W command    //
//                                                                          //

void processAzElNumeric(char character) {
  switch(_gs232AzElIndex) {
    case 0: { // first azimuth character
      _azimuthTemp =(character - 48) * 100;
      _gs232AzElIndex++;
      break;
    } 
        
    case 1: {
      _azimuthTemp = _azimuthTemp + (character - 48) * 10;
      _gs232AzElIndex++;
      break;
    } 
        
    case 2: { // final azimuth character
      _azimuthTemp = _azimuthTemp + (character - 48);
      _gs232AzElIndex++;
            
            // check for valid azimuth 
      if ((_azimuthTemp) > _maxRotorAzimuth) {
        _gs232WActive = false;
        _newAzimuth = 0L;
        _newElevation = 0L;
      }           
      break;
    }  
        
    case 3: { // first elevation character
      _elevationTemp =(character - 48) * 100;
      _gs232AzElIndex++;
      break;
    } 
        
    case 4: {
      _elevationTemp = _elevationTemp + (character - 48) * 10;
      _gs232AzElIndex++;
      break;
    } 
        
    case 5: { // last elevation character
      _elevationTemp = _elevationTemp + (character - 48);
      _gs232AzElIndex++;
            
      // check for valid elevation 
      if ((_elevationTemp) > _maxRotorElevation) {
        _gs232WActive = false;
        _newAzimuth = 0L;
        _newElevation = 0L;
      } else { // both azimuth and elevation are ok
        // set up for rotor move


        //Serial.println("AZ/EL OK");              
        _newAzimuth = _azimuthTemp;
        _newElevation = _elevationTemp;
        _azimuthMove = true;
        _elevationMove = true;
        //checkOverlap = true;
      }            
      break;
    }             
          
    default: {
      // should never get here
    }         
  } 
}


//                                                                          //
//                            EEPROM Read/Write                             //
//                                                                          //

// Write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

// Read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address) {
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}



String convertDegreeToCardinalDirection(int directionInDegrees){
    String cardinalDirection = "?";
    if( (directionInDegrees >= 348.75) && (directionInDegrees <= 360) ||
        (directionInDegrees >= 0) && (directionInDegrees <= 11.25)    ){
        cardinalDirection = "N";
    } else if( (directionInDegrees >= 11.25 ) && (directionInDegrees <= 33.75)){
        cardinalDirection = "NNE";
    } else if( (directionInDegrees >= 33.75 ) &&(directionInDegrees <= 56.25)){
        cardinalDirection = "NE";
    } else if( (directionInDegrees >= 56.25 ) && (directionInDegrees <= 78.75)){
        cardinalDirection = "ENE";
    } else if( (directionInDegrees >= 78.75 ) && (directionInDegrees <= 101.25) ){
        cardinalDirection = "E";
    } else if( (directionInDegrees >= 101.25) && (directionInDegrees <= 123.75) ){
        cardinalDirection = "ESE";
    } else if( (directionInDegrees >= 123.75) && (directionInDegrees <= 146.25) ){
        cardinalDirection = "SE";
    } else if( (directionInDegrees >= 146.25) && (directionInDegrees <= 168.75) ){
        cardinalDirection = "SSE";
    } else if( (directionInDegrees >= 168.75) && (directionInDegrees <= 191.25) ){
        cardinalDirection = "S";
    } else if( (directionInDegrees >= 191.25) && (directionInDegrees <= 213.75) ){
        cardinalDirection = "SSW";
    } else if( (directionInDegrees >= 213.75) && (directionInDegrees <= 236.25) ){
        cardinalDirection = "SW";
    } else if( (directionInDegrees >= 236.25) && (directionInDegrees <= 258.75) ){
        cardinalDirection = "WSW";
    } else if( (directionInDegrees >= 258.75) && (directionInDegrees <= 281.25) ){
        cardinalDirection = "W";
    } else if( (directionInDegrees >= 281.25) && (directionInDegrees <= 303.75) ){
        cardinalDirection = "WNW";
    } else if( (directionInDegrees >= 303.75) && (directionInDegrees <= 326.25) ){
        cardinalDirection = "NW";
    } else if( (directionInDegrees >= 326.25) && (directionInDegrees <= 348.75) ){
        cardinalDirection = "NNW";
    } else {
        cardinalDirection = "?";
    }
 
    return cardinalDirection;
}
