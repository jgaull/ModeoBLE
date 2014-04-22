/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

#ifndef ModeoBLE_h
#define ModeoBLE_h

#include "Arduino.h"

#define RESOLUTION 10

struct point {
    byte x;
    byte y;
};

struct Bezier {
    byte identifier;
    byte numPoints;
    point points[4];
    point cache[RESOLUTION];
    boolean cacheIsValid;
    byte maxX;
    byte maxY;
};

class ModeoBLE {
    
    public:
    
    ModeoBLE(byte numProperties, byte numSensors, byte numBeziers);
    
    void startup();
    void shutdown();
    void update();
    
    void registerProperty(byte identifier, bool eepromSave);
    //void registerPropertyWithCallback(byte identifier, byte readWritePermissions, void (*callback)(unsigned short, unsigned short));
    void setValueForProperty(unsigned short value, byte propertyId);
    unsigned short getValueForProperty(byte identifier);
    
    void registerSensor(byte identifier);
    void setValueForSensor(unsigned short value, byte identifier);
    unsigned short getValueForSensor(byte identifier);
    
    void registerBezier(byte identifier);
    Bezier getBezier(byte identifier);
    
    void clearEEPROM();
    void saveValueForProperty(unsigned short value, byte identifier);
    
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
    void addBezier();
    void writeBezier();
    void setProperty();
    void writeProperty();
    
    void retrieveCalibrations();
    void storeCalibrations();
    
};

#endif