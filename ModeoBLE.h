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
    
    ModeoBLE();
    
    void startup();
    void shutdown();
    void update();
    
    void getValueForProperty(byte identifier, byte *length, byte *data);
    void setValueForProperty(byte identifier, byte data[]);
    
    void setUnsignedShortValueForProperty(unsigned short value, byte identifier);
    unsigned short getUnsignedShortValueForProperty(byte identifier);
    byte getByteValueForProperty(byte identifier);
    
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
    void setProperty();
    void writeProperty();
    void setConfig();
    void writeConfig();
    
    void retrieveCalibrations();
    void storeCalibrations();
    
    void loadConfiguration();
    void registerProperty(byte identifier, byte size, bool eepromSave);
    void registerSensor(byte identifier);
    
};

#endif