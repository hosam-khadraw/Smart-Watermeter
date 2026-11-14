#include <Wire.h> // I2C Arduino Library
#include <HMC5883L_Simple.h>

// I2C Address for The HMC5883
#define addr 0x1E 



//////////// BSP.h(Board Support Package) ///////////////////
//HMC5883L
#define addr 0x1E 

#define IS_FRONT_MOUNT true
#define IS_FRONT_DESCENT false
#define IS_LISTENING true
#define IS_TRIGGERED false
//signal treatment
#define tableSize 50  
///////////////////////////////////////////////////////////

//components
/////////////////// HMC5883/////////////////////////////////
//properities
int x, y, z; 
//methods
void writeDataHMC5883();
void readDataHMC5883();
/////////////////////////////////////////////////////////////

/////////////////// Compass.h ///////////////////////////////
//properities
float heading;
HMC5883L_Simple Compass;

////////////////////////////////////////////////////////////



////////////////// SmoothingAdapter.h ///////////////////
//properities
struct MA_FilterProps {
  const int numReadings = tableSize; 
  int readings[tableSize];  
  int index = 0;              
  long total = 0.0;             
  int average = 0.0;            
};
MA_FilterProps ma_filterProps;

float smoothed_x;
float smoothed_y;
float smoothed_heading;

//methods
float MA_Filter(float sensorValue);
///////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////// SignalTreatement.h  ///////////////////////////////
//properities
enum SignalStates{
  INIT,
  FRONT_MOUNT,
  FRONT_DESCENT,
  STABLE 
};
SignalStates signal_states = FRONT_MOUNT;
SignalStates pervious_signal_state = FRONT_DESCENT;


float xTab[tableSize];
float yTab[tableSize];
float headingTab[tableSize];
bool signalStatesTab[] = {false, false, false};


int counter = 0;
int counterXTabShiftting = 0;
int meanXTabValue = 0;
int previousMeanXTabValue = 0;

bool edgeListener = IS_LISTENING;
bool isFirstShifting = true;


//methods
void table_shifting(float (&tab)[tableSize], float newValue);
void FinitStateMachine();
bool checkValInRange(float valToCheck, float valRange);

//////////////////////////////////////////////////////////////////////////




//////////////////// main /////////////////////////////////////
void setup() {
    //Serial
    Serial.begin(9600);
    //i2c
    Wire.begin();
    //HMC5883 config 
    Wire.beginTransmission(addr); // Start talking
    Wire.write(0x02); // Set the Register
    Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
    Wire.endTransmission();
    //compass config
    Compass.SetDeclination(-0, 23, 'W');
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_130);
    Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
    Serial.println("setup is done");
}

void loop() {
  
    // begin writting data
    //writeDataHMC5883();
    Wire.beginTransmission(addr);
    Wire.write(0x03); 
    Wire.endTransmission();
    // read data HMC5883
    //readDataHMC5883();
    Wire.requestFrom(addr, 6);
    if (6 <= Wire.available()) {
        x = Wire.read() << 8; // MSB x
        x |= Wire.read(); // LSB x
        z = Wire.read() << 8; // MSB z
        z |= Wire.read(); // LSB z
        y = Wire.read() << 8; // MSB y
        y |= Wire.read(); // LSB y
    }
    
    //read data compass
    heading = Compass.GetHeadingDegrees();
    //smoothing signals
    smoothed_x = MA_Filter(abs(x));
    smoothed_y = MA_Filter(abs(y));
    smoothed_heading = MA_Filter(abs(heading));
    // shift tables
    table_shifting(xTab, smoothed_x);
    //table_shifting(yTab, smoothed_y);
    //table_shifting(headingTab, smoothed_heading);
    // count steps
    if(counterXTabShiftting == tableSize){
      meanXTabValue = xTab[0];
      
      for(int i= tableSize-1; i < tableSize; i++){
          meanXTabValue = meanXTabValue + (xTab[i] - xTab[i]);
      }
      
      FinitStateMachine();
      
      previousMeanXTabValue = meanXTabValue;
      if (signalStatesTab[0] == IS_FRONT_DESCENT && signalStatesTab[1] == IS_FRONT_MOUNT && signalStatesTab[2] == IS_FRONT_DESCENT && edgeListener == IS_LISTENING) {
        counter++;
        edgeListener = IS_TRIGGERED;  
      }
      counterXTabShiftting = 0;
    }
    // Show Values
    //Serial.print("X: ");
    //Serial.print(smoothed_x);
    
    Serial.print(x);
    //Serial.print(" Y: ");
    //Serial.print(smoothed_y);
    //Serial.print(" Heading: ");
    //Serial.println(smoothed_heading);
    //Serial.println(counter);
    //for(int i=0; i < 3;i++){
      //Serial.print(signalStatesTab[i]);
    //}
    Serial.println("");
    //delay(10);
}

//////////////////////////////////////////////////////////////////


//methods HMC5883
void writeDataHMC5883(){
    Wire.beginTransmission(addr);
    Wire.write(0x03); // Start with register 3
    Wire.endTransmission();
}
void readDataHMC5883(){
    Wire.requestFrom(addr, 6);
    if (6 <= Wire.available()) {
        x = Wire.read() << 8; // MSB x
        x |= Wire.read(); // LSB x
        z = Wire.read() << 8; // MSB z
        z |= Wire.read(); // LSB z
        y = Wire.read() << 8; // MSB y
        y |= Wire.read(); // LSB y
    }
}
////////// SignalSmoothing ///////////////
float MA_Filter(float sensorValue) {
  // Subtract the last reading
  ma_filterProps.total = ma_filterProps.total - ma_filterProps.readings[ma_filterProps.index];
  // Add the new reading
  ma_filterProps.readings[ma_filterProps.index] = sensorValue;
  ma_filterProps.total = ma_filterProps.total + ma_filterProps.readings[ma_filterProps.index];
  // Advance to the next position in the array
  ma_filterProps.index = (ma_filterProps.index + 1) % ma_filterProps.numReadings;
  // Calculate the average
  ma_filterProps.average = ma_filterProps.total / ma_filterProps.numReadings; 
  return ma_filterProps.average; 
}


///////// SignalTreatment //////////////
bool checkValInRange(float valToCheck, float valRange){
    valToCheck = (int)valToCheck;
    valRange = (int)valRange;
    return (valToCheck == valRange || valToCheck == valRange + 1 || valToCheck == valRange + 2 || valToCheck == valRange + 3 || valToCheck == valRange - 1 || valToCheck == valRange - 2 || valToCheck == valRange - 3);
}
void table_shifting(float (&tab)[tableSize], float newValue){
    if(signal_states == INIT && !isFirstShifting){
        isFirstShifting = true;
        for(int i = 0; i < tableSize; i++){
            tab[i] = newValue;
        }
    }
    else{
        for (int i = tableSize; i > 0; i--) {
            tab[i] = tab[i - 1];
        }
        tab[0] = newValue;
        counterXTabShiftting++;
    }
}

void FinitStateMachine(){
    switch(signal_states){
        case STABLE:
            if ( meanXTabValue < previousMeanXTabValue ) {
              if(pervious_signal_state == FRONT_DESCENT){
                signalStatesTab[2] = signalStatesTab[1];  
                signalStatesTab[1] = signalStatesTab[0];
                signalStatesTab[0] = IS_FRONT_MOUNT;
                edgeListener = IS_LISTENING; 
                signal_states = FRONT_MOUNT;
              }
            } 
            else if (meanXTabValue > previousMeanXTabValue) {
              if(pervious_signal_state == FRONT_MOUNT){
                signalStatesTab[2] = signalStatesTab[1];  
                signalStatesTab[1] = signalStatesTab[0];
                signalStatesTab[0] = IS_FRONT_DESCENT;
                edgeListener = IS_LISTENING;
                signal_states = FRONT_DESCENT;
              }
            }
        break;
        case FRONT_MOUNT:
            if(checkValInRange(meanXTabValue, previousMeanXTabValue)){
              signal_states = STABLE;
            }
            else if (meanXTabValue < previousMeanXTabValue){
                if(pervious_signal_state == FRONT_DESCENT){
                  signalStatesTab[2] = signalStatesTab[1];  
                  signalStatesTab[1] = signalStatesTab[0];
                  signalStatesTab[0] = IS_FRONT_DESCENT;
                  signal_states = FRONT_DESCENT;
                }   
            }
        break;

        case FRONT_DESCENT:
            if(checkValInRange(meanXTabValue, previousMeanXTabValue)){
              signal_states = STABLE;
            }
            else if (meanXTabValue > previousMeanXTabValue){
                if(pervious_signal_state == FRONT_MOUNT){
                  signalStatesTab[2] = signalStatesTab[1];  
                  signalStatesTab[1] = signalStatesTab[0];
                  signalStatesTab[0] = IS_FRONT_MOUNT;
                  edgeListener = IS_LISTENING;
                  signal_states = FRONT_MOUNT;
                }
            }
        break;

    }
}
