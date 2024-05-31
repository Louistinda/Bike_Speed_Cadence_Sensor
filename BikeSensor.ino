/*
Multi BLE Sensor - Richard Hedderly 2019

Based on heart sensor code by Andreas Spiess which was based on a Neil
Kolban example.

Based on Neil Kolban example for IDF:
https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
Ported to Arduino ESP32 by Evandro Copercini
updates by chegewara

*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

uint16_t crankrev;  // Cadence RPM
uint16_t lastcrank; // Last crank time

uint32_t wheelrev;      // Wheel revolutions
uint16_t lastwheel;     // Last crank time

uint16_t cadence;

byte speedkph;    // Speed in KPH
byte speedmph;    // Speed in MPH
byte cscmeasurement[11] = { 0b00000011, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte cscfeature[1] = { 0b0000000000000010 };
byte sensorlocation[1] = { 6 };

bool _BLEClientConnected = false;

// Define Speed and Cadence Properties
#define speedService BLEUUID ((uint16_t) 0x1816)

// 0x2A5B - Notify - Client Charactaristic Config is 1 - DONE : CSC Measurement
// 0x2A5C - Read - 0x200 - DONE : CSC Feature
// 0x2A5D - Read - None or 0x06 when active - Done : Sensor Location - 6 is right crank
// 0x2A55 - Write Indicate - Client Characteristic Config is 2 : SC Control Point

//CSC Measurement Characteristic
BLECharacteristic cscMeasurementCharacteristics( BLEUUID ((uint16_t) 0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
//CSC Feature Characteristic
BLECharacteristic cscFeatureCharacteristics(     BLEUUID ((uint16_t) 0x2A5C), BLECharacteristic::PROPERTY_READ);
//Sensor Location Characteristic
BLECharacteristic sensorLocationCharacteristics( BLEUUID ((uint16_t) 0x2A5D), BLECharacteristic::PROPERTY_READ);
// SC Control Point Characteristic
BLECharacteristic scControlPointCharacteristics( BLEUUID ((uint16_t) 0x2A55), BLECharacteristic::PROPERTY_WRITE | 
                                                                              BLECharacteristic::PROPERTY_INDICATE);

//0x2901 is a custom user description
BLEDescriptor cscMeasurementDescriptor( BLEUUID ((uint16_t) 0x2901));
BLEDescriptor cscFeatureDescriptor(     BLEUUID ((uint16_t) 0x2901));
BLEDescriptor sensorLocationDescriptor( BLEUUID ((uint16_t) 0x2901));
BLEDescriptor scControlPointDescriptor( BLEUUID ((uint16_t) 0x2901));

// Define Battery
//Servicec UUID 0x180F - Battery Level UUID 0x2A19

// Define Firmware
//UUID 0x2A26

class MyServerCallbacks:public BLEServerCallbacks
{
  void onConnect (BLEServer * pServer)
  {
    _BLEClientConnected = true;
  };

  void onDisconnect (BLEServer * pServer)
  {
    _BLEClientConnected = false;
  }

};

void InitBLE ()
{
  // Create BLE Device
  BLEDevice::init ("Multi Fitness BLE Sensor");

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer ();
  pServer->setCallbacks( new MyServerCallbacks ());

  // Create Speed and Cadence Configuration
  BLEService *pSpeed = pServer->createService(speedService);

  // Create  Speed and Cadence Service
  pSpeed->addCharacteristic( &cscMeasurementCharacteristics);
  pSpeed->addCharacteristic( &cscFeatureCharacteristics);
  pSpeed->addCharacteristic( &sensorLocationCharacteristics);
  pSpeed->addCharacteristic( &scControlPointCharacteristics);
  
  cscMeasurementDescriptor.setValue( "Exercise Bike CSC Measurement");
  cscMeasurementCharacteristics.addDescriptor( &cscMeasurementDescriptor);
  cscMeasurementCharacteristics.addDescriptor( new BLE2902());

  cscFeatureDescriptor.setValue ("Exercise Bike CSC Feature");
  cscFeatureCharacteristics.addDescriptor (&cscFeatureDescriptor);
 
  sensorLocationDescriptor.setValue( "Exercise Bike CSC Sensor Location");
  sensorLocationCharacteristics.addDescriptor( &sensorLocationDescriptor);
 
  scControlPointDescriptor.setValue( "Exercise Bike CSC SC Control Point");
  scControlPointCharacteristics.addDescriptor( &scControlPointDescriptor);

// Add UUIDs for Services to BLE Service Advertising
  pServer->getAdvertising()->addServiceUUID( speedService);

// Start p Instances
  pSpeed->start();

// Start Advertising
  pServer->getAdvertising()->start();
}

void setup ()
{
  Serial.begin(115200);
  Serial.println("Start");
  InitBLE();
  crankrev = 0;
  lastcrank = 0;
  wheelrev = 0;
  lastwheel = 0;
  
}

void loop ()
{
// _______________________
// SPEED + CADENCE SECTION
// -----------------------
// Read Cadence RPM Pins

  cadence = 50; // Some arbitrary test value
  speedkph = 1024;
  
  if (cadence == 0) 
  {
     cadence = 1;
  }
  crankrev = crankrev + 1;
  lastcrank = lastcrank + 1024*60/cadence;

  wheelrev = wheelrev + 1;
  lastwheel = lastwheel + 1024*60/speedkph;



// That, lastcran in the array position 2 will only show FF or 256 even
// if lastcrank is set to a uint16_t


// Set and Send Speed and Cadence Notify
  cscmeasurement[1] = wheelrev & 0xFF;
  cscmeasurement[2] = (wheelrev >> 8) & 0xFF; 

  cscmeasurement[3] = (wheelrev >> 16) & 0xFF; 
  cscmeasurement[4] = (wheelrev >> 24) & 0xFF; 
  
  cscmeasurement[5] = lastwheel & 0xFF;
  cscmeasurement[6] = (lastwheel >> 8) & 0xFF; 

  cscmeasurement[7] = crankrev & 0xFF;
  cscmeasurement[8] = (crankrev >> 8) & 0xFF; 

  cscmeasurement[9] = lastcrank & 0xFF;
  cscmeasurement[10] = (lastcrank >> 8) & 0xFF; 
  
  cscMeasurementCharacteristics.setValue(cscmeasurement, 11);
  cscMeasurementCharacteristics.notify();

  cscFeatureCharacteristics.setValue(cscfeature, 1);
  sensorLocationCharacteristics.setValue(sensorlocation, 1); 

  Serial.print("crankrev = ");
  Serial.println(crankrev);
  Serial.print("lastcrank = ");
  Serial.println(lastcrank);

  Serial.print(cscmeasurement[0]);  //Should be 2 to reflect binary - Yes
  Serial.print(" ");
  Serial.print(cscmeasurement[1]);  // Should increment by 2 - Yes
  Serial.print(" ");
  Serial.print(cscmeasurement[2]);
  Serial.print(" ");
  Serial.print(cscmeasurement[3]);
  Serial.print(" ");
  Serial.print(cscmeasurement[4]);
  Serial.print(" ");
  Serial.print(cscmeasurement[5]);
  Serial.print(" ");
  Serial.print(cscmeasurement[6]);
  Serial.print(" ");
  Serial.print(cscmeasurement[7]);
  Serial.print(" ");
  Serial.print(cscmeasurement[8]);
  Serial.print(" ");
  Serial.print(cscmeasurement[9]);
  Serial.print(" ");
  Serial.print(cscmeasurement[10]);
  Serial.print(" ");
  Serial.print(cscmeasurement[11]);
  Serial.println(" ");
  delay(1000);
}
