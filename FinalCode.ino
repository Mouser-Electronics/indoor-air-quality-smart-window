#include <Arduino.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;

//Declaring Motor Variables
const int IS_1 = A0; 
const int IN_1 = 3; 
const int INH_1 = 14; 

const int IS_2 = A1; 
const int IN_2 = 10; 
const int INH_2 = 13; 

bool window_state = false;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];
    
    
    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

SensirionI2CScd4x scd4x;

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}
void printSerialNumber2(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}


void setup() {
  // Initialize serial and wait for port to open:
  //Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
  
  delay(1500); 

  Wire.begin();
  
  sen5x.begin(Wire);
  scd4x.begin(Wire);
  
  
    uint16_t error;
    uint16_t error2;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
    // stop potentially previously started measurement
    error2 = scd4x.stopPeriodicMeasurement();
    if (error2) {
        Serial.print("Error CO2 trying to execute stopPeriodicMeasurement(): ");
        errorToString(error2, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
  // Print SEN55 module information if i2c buffers are large enough
  #ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
  #endif
  
    // set a temperature offset in degrees celsius
    // Note: supported by SEN54 and SEN55 sensors
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used
    // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
    // `setRhtAccelerationMode`.
    //
    // Adjust tempOffset to account for additional temperature offsets
    // exceeding the SEN module's self heating.
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }
  
    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error2 = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error2) {
        Serial.print("Error CO2 trying to execute getSerialNumber(): ");
        errorToString(error2, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber2(serial0, serial1, serial2);
    }
  
    // Start Measurement CO2
    error2 = scd4x.startPeriodicMeasurement();
    if (error2) {
        Serial.print("Error CO2 trying to execute startPeriodicMeasurement(): ");
        errorToString(error2, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
  Serial.println("Waiting for first measurement... (5 sec)");
  
  //Motor Pins Setup
  pinMode(IS_1, INPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(INH_1, OUTPUT);
  pinMode(IS_2, INPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(INH_2, OUTPUT);

  analogWrite(IN_1, 0); 
  digitalWrite(INH_1, HIGH); 
  analogWrite(IN_2, 0); 
  digitalWrite(INH_2, HIGH); 
}


void loop() {
  
  uint16_t error;
  uint16_t error2;
  
  char errorMessage[256];

  delay(1000);

  // Read Measurement
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  error = sen5x.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
      noxIndex);

  if (error) {
      Serial.print("Error trying to execute readMeasuredValues(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("MassConcentrationPm1p0:");
      Serial.print(massConcentrationPm1p0);
      Serial.print("\t");
      Serial.print("MassConcentrationPm2p5:");
      Serial.print(massConcentrationPm2p5);
      Serial.print("\t");
      Serial.print("MassConcentrationPm4p0:");
      Serial.print(massConcentrationPm4p0);
      Serial.print("\t");
      Serial.print("MassConcentrationPm10p0:");
      Serial.print(massConcentrationPm10p0);
      Serial.print("\t");
      Serial.print("AmbientHumidity:");
      if (isnan(ambientHumidity)) {
          Serial.print("n/a");
      } else {
          Serial.print(ambientHumidity);
      }
      Serial.print("\t");
      Serial.print("AmbientTemperature:");
      if (isnan(ambientTemperature)) {
          Serial.print("n/a");
      } else {
          Serial.print(ambientTemperature);
      }
      Serial.print("\t");
      Serial.print("VocIndex:");
      if (isnan(vocIndex)) {
          Serial.print("n/a");
      } else {
          Serial.print(vocIndex);
      }
      Serial.print("\t");
      Serial.print("NoxIndex:");
      if (isnan(noxIndex)) {
          Serial.println("n/a");
      } else {
          Serial.println(noxIndex);
      }
  }
  
  // CO2
   delay(5000);
    

  // Read Measurement
  uint16_t co2;
  //int co2;
  float temperature;
  float humidity;
  error2 = scd4x.readMeasurement(co2, temperature, humidity);
  
  if (error2) {
      Serial.print("Error CO2 trying to execute readMeasurement(): ");
      errorToString(error2, errorMessage, 256);
      Serial.println(errorMessage);
  } else if (co2 == 0) {
      Serial.println("Invalid sample detected, skipping.");
  } else {
      Serial.print("Co2:");
      Serial.print(co2);
      Serial.print("\t");
      Serial.print("Temperature:");
      Serial.print(temperature);
      Serial.print("\t");
      Serial.print("Humidity:");
      Serial.println(humidity);
  }
  
  
 ///////////////Printing IAQ Message/////////////////
  if(co2<=1000){
    Serial.println("####Good Indoor Air Quality####");
  }
  
  else if(co2>1000 && co2<1300){
    Serial.println("******Bad Indoor Air Quality******");
  } 
  
  else if(co2>=1300){
  Serial.print("!!!!!Very Bad Indoor Air Quality!!!!!");
  }
  //////////////////////////////////////////////////


if (window_state == false && co2>=1300){
    biDirectionPower(-100);
    delay(5000);
    biDirectionPower(0);
    window_state = true;
  }
  else if (window_state == true && co2<=1250){
    biDirectionPower(100);
    delay(5000);
    biDirectionPower(0);
    window_state = false;
  }
}


//Motor Direction Function
void biDirectionPower(int inputBiPower){
  if(inputBiPower == 0 ) {
    
    analogWrite(IN_1, 0);
    analogWrite(IN_2, 0);
   }
  else if(inputBiPower>0){
    analogWrite(IN_1,inputBiPower);
  }
  else{
    analogWrite(IN_2,abs(inputBiPower));
    }
}

