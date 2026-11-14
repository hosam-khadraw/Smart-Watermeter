#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>



//reading compass  
float heading;
HMC5883L_Simple Compass;

void table_shiffting(float &tab[4], float newValue){
  tab[3] = tab[2];
  tab[2] = tab[1];
  tab[1] = tab[0];
  tab[0] = newValue;
}

//smoothing 
float smoothed_signal;
struct MA_FilterProps {
  const int numReadings = 10; 
  int readings[10];  
  int index = 0;              
  long total = 0.0;             
  int average = 0.0;            
};
MA_FilterProps ma_filterProps;
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


//filtering
float filter_low = 222.0;
float filtre_high = 235.0;
float filtred_signal[4];
bool isFirstFilter = false;
float passBand_Filter(){
  if(smoothed_signal <= 235.0){
    table_shiffting(filtred_signal, 235.0);
  }
  else if(smoothed_signal >= 222.0){
    table_shiffting(filtred_signal, 222.0);
  }
  else if(smoothed_signal > 222.0 && smoothed_signal < 235.0 ){
    table_shiffting(filtred_signal, smoothed_signal);
  }
}


//float echelon_des_valeurs[4];
//float filtered_signal[4]; 

//transforming
float transformed_signal;
float transformed_high = 235.0;
float transformed_low = 222.0;
enum EtatSignale{
  INIT,
  FRONT_MONT,
  FRONT_DESCENT,
  UNKNOWN_STATE 
};
EtatSignale etats_signale = INIT;

float rectangular_signal(){
  if(etats_signale == FRONT_MONT){
    transformed_signal = transformed_high;
  }
  else if(etats_signale == FRONT_DESCENT){
    transformed_signal = transformed_low;
  }
}

  // Initialize the arrays with some values
  // echelon_des_valeurs[0] = 100.0;
  // echelon_des_valeurs[1] = 120.0;
  // echelon_des_valeurs[2] = 140.0;
  // echelon_des_valeurs[3] = 160.0;

  //filtered_signal[0] = 150.0;
  //filtered_signal[1] = 130.0;
  //filtered_signal[2] = 110.0;
  //filtered_signal[3] = 90.0;


void setup() {
  //init serial
  Serial.begin(9600);
  //init i2c
  Wire.begin();
  //init magnometer
  Compass.SetDeclination(-0, 23, 'W'); 
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  // Initialize all readings to 0
  for (int i = 0; i < ma_filterProps.numReadings; i++) {
    ma_filterProps.readings[i] = 0.0;
  }
}

void loop() {
  //lire signale
  heading = Compass.GetHeadingDegrees();
  
  //lissage de signale
  smoothed_signal = MA_Filter(heading);
  
  //filtrage de signale
  if(etats_signale == INIT && isFirstFilter == false){
    isFirstFilter = true;
    for(int i = 0; i < 4; i++){
      filtered_signal[i] = smoothed_signal;
    }
  }
  filtred_signal = passBand_Filter(); 
  
  //transformation du 
  switch(etats_signale){
    case INIT:
      if(echelon_des_valeurs[0] < filter_low && 
         echelon_des_valeurs[0] <= echelon_des_valeurs[1] && 
         filtered_signal[1] <= echelon_des_valeurs[2] && 
         filtered_signal[2] <= echelon_des_valeurs[3]) {
        etats_signale = FRONT_DESCENT;
      } 
      else if(echelon_des_valeurs[0] > filter_low && 
                echelon_des_valeurs[0] >= echelon_des_valeurs[1] && 
                filtered_signal[1] >= echelon_des_valeurs[2] && 
                filtered_signal[2] >= echelon_des_valeurs[3]) {
        etats_signale = FRONT_MONT;
      }
      break;

    case FRONT_MONT:
      if(echelon_des_valeurs[0] < filter_high && 
         echelon_des_valeurs[0] <= echelon_des_valeurs[1] && 
         filtered_signal[1] <= echelon_des_valeurs[2] && 
         filtered_signal[2] < echelon_des_valeurs[3]) {
        etats_signale = FRONT_DESCENT;
      }
      break;

    case FRONT_DESCENT:
      if(echelon_des_valeurs[0] > filter_high && 
         echelon_des_valeurs[0] >= echelon_des_valeurs[1] && 
         filtered_signal[1] >= echelon_des_valeurs[2] && 
         filtered_signal[2] > echelon_des_valeurs[3]) {
        etats_signale = FRONT_MONT;
      }
      break;
  }
  
  Serial.print("\tsmoothed_signal: ");
  Serial.println(smoothed_signal);

  delay(1000);
}
