/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

#ifndef ModeoBLE_h
#define ModeoBLE_h

#include "Arduino.h"

#define RESOLUTION 10

typedef void (Callback) (byte length, byte value[]);

class ModeoBLE {
    
    public:
    
    ModeoBLE(byte numProperties, byte numSensors);
    
    void startup();
    void shutdown();
    void update();
    
    void registerProperty(byte identifier, byte size, bool eepromSave);
    void registerPropertyWithCallback(byte identifier, byte size, bool eepromSave, Callback *callback);
    
    void getValueForProperty(byte identifier, byte *length, byte *data);
    void setValueForProperty(byte identifier, byte data[]);
    
    void setUnsignedShortValueForProperty(unsigned short value, byte identifier);
    unsigned short getUnsignedShortValueForProperty(byte identifier);
    byte getByteValueForProperty(byte identifier);
    
    void registerSensor(byte identifier);
    void setValueForSensor(unsigned short value, byte identifier);
    unsigned short getValueForSensor(byte identifier);
    
    //void registerBezier(byte identifier);
    //Bezier getBezier(byte identifier);
    
    //void clearEEPROM();
    //void saveValueForProperty(unsigned short value, byte identifier);
    
    private:
    
    int indexForProperty(byte identifier);
    int indexForSensor(byte identifier);
    int indexForBezier(byte identifier);
    
    void performBluetoothReceive();
    void performConnect();
    void performDisconnect();
    void getPropertyValue();
    void writeGetProperty();
    void clearBLEBuffer();
    void getSensorValue();
    //void addBezier();
    //void writeBezier();
    void setProperty();
    void writeProperty();
    
    void retrieveCalibrations();
    void storeCalibrations();
    
};

#endif