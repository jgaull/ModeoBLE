/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

//Sensors
#define SENSOR_RIDER_EFFORT 0
#define SENSOR_CURRENT_STRAIN 1
#define SENSOR_SPEED 2
#define SENSOR_RAW_STRAIN 3
#define SENSOR_TORQUE_APPLIED 4
#define SENSOR_MOTOR_TEMP 5
#define SENSOR_BATTERY_VOLTAGE 6
#define SENSOR_FILTERED_RIDER_EFFORT 7

#define NUM_SENSORS 8 //Is there a better way to do this?

//Request Identifiers
#define REQUEST_CONNECT 10
#define REQUEST_DISCONNECT 11
#define REQUEST_GET_PROPERTY_VALUE 12
#define REQUEST_ADD_BEZIER 14
#define REQUEST_GET_SENSOR_VALUE 15
#define REQUEST_WRITE_BEZIER 17
#define REQUEST_WRITE_GET_PROPERTY 18
#define REQUEST_SET_PROPERTY 19
#define REQUEST_WRITE_PROPERTY 20

#define STATE_OFF 0
#define STATE_ON 1

#include <AltSoftSerial.h>
#include <EEPROM.h>

#include "Arduino.h"
#include "ModeoBLE.h"

struct Property {
    unsigned short value;
    byte identifier;
    byte readWritePermissions;
    bool pendingSave;
    bool eepromSave;
    bool callbackOnChange;
    Callback *callback;
};

struct Sensor {
    byte identifier;
    unsigned short value;
};

Property _properties[16];
byte _propertiesLength = 0;
byte _numProperties = 0;
Property _propertyPendingSave;
byte _propertyIdentifierForPropertyPendingSave;

Sensor _sensors[8];
byte _sensorsLength = 0;
byte _numSensors = 0;

Bezier _beziers[2];
byte _beziersLength = 0;
byte _numBeziers = 0;
Bezier _bezierPendingSave;

byte previousWriteRequest[16];
byte previousWriteRequestLength = 0;

AltSoftSerial _bleMini;


byte _lastAvailable = 0;

byte _state = STATE_OFF;

ModeoBLE::ModeoBLE(byte numProperties, byte numSensors, byte numBeziers) {
    Serial.println("ModeoBLE Constructed.");
    
    _numProperties = numProperties;
    _numSensors = numSensors;
    _numBeziers = numBeziers;
}

void ModeoBLE::startup() {
    Serial.println("ModeoBLE Startup!");
    
    if (_state == STATE_OFF && _numProperties == _propertiesLength && _numSensors == _sensorsLength) {
        retrieveCalibrations();
        
        _bleMini.begin(57600);
    }
    else {
        Serial.println("Failure at startup.");
    }
}

void ModeoBLE::shutdown() {
    Serial.println("ModeoBLE Shutdown!");
    storeCalibrations();
    _state = STATE_OFF;
}

void ModeoBLE::update() {
    //Serial.println("ModeoBLE Update");
    performBluetoothReceive();
}

void ModeoBLE::registerProperty(byte identifier, bool eepromSave) {
    
    if (indexForProperty(identifier) == -1) {
        _properties[_propertiesLength].identifier = identifier;
        _properties[_propertiesLength].readWritePermissions = 0;
        _properties[_propertiesLength].pendingSave = false;
        _properties[_propertiesLength].eepromSave = eepromSave;
        _properties[_propertiesLength].callbackOnChange = false;
        
        _propertiesLength++;
        
        Serial.println("Property Registered.");
    }
    else {
        Serial.println("Property has already been registered");
    }
}

void ModeoBLE::registerPropertyWithCallback(byte identifier, bool eepromSave, Callback *callback) {
    if (indexForProperty(identifier) == -1) {
        _properties[_propertiesLength].identifier = identifier;
        _properties[_propertiesLength].readWritePermissions = 0;
        _properties[_propertiesLength].pendingSave = false;
        _properties[_propertiesLength].eepromSave = eepromSave;
        _properties[_propertiesLength].callback = callback;
        _properties[_propertiesLength].callbackOnChange = true;
        
        (*_properties[_propertiesLength].callback)(0,0);
        
        _propertiesLength++;
        
        Serial.println("Property registered with callback.");
    }
    else {
        Serial.println("Property has already been registered with callback.");
    }
}

void ModeoBLE::setValueForProperty(unsigned short value, byte identifier) {
    Serial.println("Set Value");
    
    int index = indexForProperty(identifier);
    if (index != -1) {
        
        if (_properties[index].value != value) {
            _properties[index].value = value;
            _properties[index].pendingSave = true;
        }
    }
}


unsigned short ModeoBLE::getValueForProperty(byte identifier) {
    int index = indexForProperty(identifier);
    if (index != -1) {
        return _properties[index].value;
    }
}

void ModeoBLE::registerSensor(byte identifier) {
    if (indexForProperty(identifier) == -1) {
        _sensors[_sensorsLength].identifier = identifier;
        _sensors[_sensorsLength].value = 0;
        _sensorsLength++;
        Serial.println("Sensor Registered.");
    }
    else {
        Serial.println("Sensor already registered.");
    }
}

void ModeoBLE::setValueForSensor(unsigned short value, byte identifier) {
    int index = indexForSensor(identifier);
    if (index != -1) {
        _sensors[index].value = value;
    }
    else {
        Serial.print("Connot set sensor value. Sensor with identifier ");
        Serial.print(identifier);
        Serial.println(" not valid.");
    }
}

unsigned short ModeoBLE::getValueForSensor(byte identifier) {
    int index = indexForSensor(identifier);
    if (index != -1) {
        return _sensors[index].value;
    }
    else {
        Serial.print("Cannot get sensor value. Sensor with identifier ");
        Serial.print(identifier);
        Serial.println(" not valid.");
    }
}

void ModeoBLE::registerBezier(byte identifier) {
    if (indexForBezier(identifier) == -1) {
        _beziers[_beziersLength].identifier = identifier;
        _beziers[_beziersLength].cacheIsValid = false;
        _beziers[_beziersLength].numPoints = 4;
        _beziers[_beziersLength].maxX = 255;
        _beziers[_beziersLength].maxY = 255;
        
        for (byte i = 0; i < 4; i++) {
            _beziers[_beziersLength].points[i].x = 0;
            _beziers[_beziersLength].points[i].y = 0;
        }
        
        _beziersLength++;
        Serial.println("Register bezier.");
    }
    else {
        Serial.print("Bezier with ID ");
        Serial.print(identifier);
        Serial.println(" already registered.");
    }
}

Bezier ModeoBLE::getBezier(byte identifier) {
    int index = indexForBezier(identifier);
    if (index != -1) {
        return _beziers[index];
    }
    else {
        Serial.print("Cannot get sensor value. Sensor with identifier ");
        Serial.print(identifier);
        Serial.println(" not valid.");
    }
}

//Debug funtions
void ModeoBLE::saveValueForProperty(unsigned short value, byte identifier) {
    int index = indexForProperty(identifier);
    if (index != -1) {
        bool eepromSave = _properties[index].eepromSave;
        
        _properties[index].value = value;
        _properties[index].eepromSave = true;
        _properties[index].pendingSave = true;
        
        storeCalibrations();
        
        _properties[index].eepromSave = eepromSave;
    }
}

void ModeoBLE::clearEEPROM() {
    Serial.println("Clear EEPROM");
}

//Private in-yer-face
int ModeoBLE::indexForProperty(byte identifier) {
    for (byte i = 0; i < _propertiesLength; i++) {
        if (_properties[i].identifier == identifier) {
            return i;
        }
    }
    
    return -1;
}

int ModeoBLE::indexForSensor(byte identifier) {
    for (byte i = 0; i < _sensorsLength; i++) {
        if (_sensors[i].identifier == identifier) {
            return i;
        }
    }
    
    return -1;
}

int ModeoBLE::indexForBezier(byte identifier) {
    for (byte i = 0; i < _beziersLength; i++) {
        if (_beziers[i].identifier == identifier) {
            return i;
        }
    }
    
    return -1;
}


//BLE Shit
void ModeoBLE::performBluetoothReceive() {
    byte currentlyAvailable = _bleMini.available();
    
    if ( currentlyAvailable > 0 && currentlyAvailable == _lastAvailable ) {
        
        //Serial.print("currentlyAvailable = ");
        //Serial.println(currentlyAvailable);
        
        byte identifier = _bleMini.read();
        
        //Serial.print("identifier: ");
        //Serial.println(identifier);
        
        switch(identifier) {
            case REQUEST_CONNECT:
                Serial.println("connect");
                performConnect();
                break;
                
            case REQUEST_DISCONNECT:
                Serial.println("disconnect");
                performDisconnect();
                break;
                
            case REQUEST_GET_PROPERTY_VALUE:
                Serial.println("get property");
                getPropertyValue();
                break;
                
            case REQUEST_ADD_BEZIER:
                Serial.println("bezier");
                addBezier();
                break;
                
            case REQUEST_GET_SENSOR_VALUE:
                Serial.println("get sensor value");
                getSensorValue();
                break;
                
            case REQUEST_WRITE_BEZIER:
                Serial.println("write bezier");
                writeBezier();
                break;
                
            case REQUEST_WRITE_GET_PROPERTY:
                Serial.println("write get property");
                writeGetProperty();
                break;
                
            case REQUEST_SET_PROPERTY:
                Serial.println("set property");
                setProperty();
                break;
                
            case REQUEST_WRITE_PROPERTY:
                Serial.println("write property");
                writeProperty();
                break;
                
            default:
                Serial.print("Uknown command: ");
                Serial.println(identifier);
        }
        
        clearBLEBuffer();
        _lastAvailable = 0;
    }
    else {
        _lastAvailable = currentlyAvailable;
    }
}

void ModeoBLE::performConnect() {
    //stopSensorUpdates();
    _bleMini.write((byte)REQUEST_CONNECT);
    _bleMini.write(1);
}

void ModeoBLE::performDisconnect() {
    storeCalibrations();
    //stopSensorUpdates();
    _bleMini.write(REQUEST_DISCONNECT);
    _bleMini.write(1);
}

void ModeoBLE::getPropertyValue() {
    if ( _bleMini.available() >= 1) {
        byte propertyIdentifier = _bleMini.read();
        
        if ( propertyIdentifier < _numProperties) {
            _bleMini.write(REQUEST_GET_PROPERTY_VALUE);
            _bleMini.write(propertyIdentifier);
            _bleMini.write(_properties[propertyIdentifier].value);
            _bleMini.write(_properties[propertyIdentifier].value >> 8);
        }
        else {
            clearBLEBuffer();
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::writeGetProperty() {
    if ( _bleMini.available() >= 3 ) {
        byte propertyIdentifier = _bleMini.read();
        byte data1 = _bleMini.read();
        byte data2 = _bleMini.read();
        unsigned short value = (data2 << 8) + data1;
        
        if ( _properties[propertyIdentifier].value == value ) {
            _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
            _bleMini.write(1);
        }
        else {
            _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
            _bleMini.write((byte)0);
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::setProperty() {
    if ( _bleMini.available() >= 1) {
        byte headerSize = 2;
        byte dataSize = 3;
        byte numProperties = _bleMini.read();
        byte numBytes = numProperties * dataSize;
        previousWriteRequestLength = headerSize + numBytes;
        
        if (_bleMini.available() >= numBytes) {
            
            previousWriteRequest[0] = REQUEST_SET_PROPERTY;
            previousWriteRequest[1] = numProperties;
            for (byte i = headerSize; i < numBytes; i++) {
                previousWriteRequest[i] = _bleMini.read();
            }
            
            for (byte i = 0; i < previousWriteRequestLength; i++) {
                _bleMini.write(previousWriteRequest[i]);
            }
        }
        else {
            clearBLEBuffer();
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::writeProperty() {
    byte headerSize = 2;
    byte dataSize = 3;
    byte numProperties = previousWriteRequest[1];
    boolean success = true;
    
    for (byte i = headerSize; i < previousWriteRequestLength; i += dataSize) {
        byte propertyIdentifier = previousWriteRequest[i];
        byte data1 = previousWriteRequest[i + 1];
        byte data2 = previousWriteRequest[i + 2];
        unsigned short value = (data2 << 8) + data1;
        
        if (propertyIdentifier < _numProperties) {
            unsigned short oldValue = _properties[propertyIdentifier].value;
            _properties[propertyIdentifier].value = value;
            _properties[propertyIdentifier].pendingSave = true;
            
            if (_properties[propertyIdentifier].callbackOnChange) {
                (*_properties[propertyIdentifier].callback)(oldValue, value);
            }
        }
        else {
            success = false;
        }
    }
    
    _bleMini.write(REQUEST_WRITE_PROPERTY);
    _bleMini.write(success);
}

void ModeoBLE::getSensorValue() {
    if (_bleMini.available() >= 1) {
        
        byte sensorIdentifier = _bleMini.read();
        
        unsigned short value = _sensors[sensorIdentifier].value;
        
        _bleMini.write(REQUEST_GET_SENSOR_VALUE);
        _bleMini.write(sensorIdentifier);
        _bleMini.write(value);
        _bleMini.write(value >> 8);
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::addBezier() {
    Bezier bezier;
    byte headerSize = 4;
    
    if ( _bleMini.available() >= headerSize ) {
        byte header[headerSize];
        
        for (int i = 0; i < headerSize; i++) {
            header[i] = _bleMini.read();
        }
        
        bezier.identifier = header[0];
        bezier.maxX = header[1];
        bezier.maxY = header[2];
        bezier.numPoints = header[3];
        bezier.cacheIsValid = false;
        
        byte bodySize = header[3] * 2;
        byte body[bodySize];
        
        if (_bleMini.available() >= bodySize) {
            
            for (byte i = 0; i < bodySize; i += 2) {
                byte pointX = _bleMini.read();
                byte pointY = _bleMini.read();
                
                body[i] = pointX;
                body[i + 1] = pointY;
                
                bezier.points[i / 2].x = pointX;
                bezier.points[i / 2].y = pointY;
            }
            
            _bezierPendingSave = bezier;
            
            byte messageData[headerSize + bodySize];
            for (byte i = 0; i < headerSize; i++) {
                messageData[i] = header[i];
            }
            
            for (byte i = 0; i < bodySize; i++) {
                messageData[i + headerSize] = body[i];
            }
            
            _bleMini.write(REQUEST_ADD_BEZIER);
            for (byte i = 0; i < headerSize + bodySize; i++) {
                _bleMini.write(messageData[i]);
            }
        }
        else {
            /*
             Serial.print("Needs ");
             Serial.print(bezier.numPoints * 2);
             Serial.print(" bytes for body. Has ");
             Serial.print(BLEMini.available());
             Serial.println(" bytes.");
             //*/
            clearBLEBuffer();
        }
    }
    else {
        //Serial.print("not enough bytes for header: ");
        //Serial.println(BLEMini.available());
        clearBLEBuffer();
    }
}

void ModeoBLE::writeBezier() {
    
    if (_bleMini.available() >= 1) {
        byte bezierType = _bleMini.read();
        
        boolean success = false;
        if (bezierType == _bezierPendingSave.identifier) {
            
            int index = indexForBezier(_bezierPendingSave.identifier);
            if (index != -1) {
                
                _beziers[index] = _bezierPendingSave;
                
                success = true;
                
                _bleMini.write(REQUEST_WRITE_BEZIER);
                _bleMini.write(success);
            }
        }
        else {
            //Serial.println("bezier type did not match. flail.");
        }
    }
    else {
        clearBLEBuffer();
    }
}

void ModeoBLE::clearBLEBuffer() {
    while(_bleMini.available() > 0) {
        _bleMini.read();
    }
}


//EEPROM Shit
void ModeoBLE::retrieveCalibrations() {
    
    //Serial.println("Started property restore");
    byte propertyCount = 0;
    for (byte i = 0; i < _numProperties; i++) {
        
        if (_properties[i].eepromSave) {
            int lsb = i * 2 + 1;
            int msb = i * 2;
            
            _properties[i].value = (EEPROM.read(msb) << 8) + EEPROM.read(lsb);
            _properties[i].pendingSave = false;
            
            propertyCount++;
        }
    }
    
    ///*
     Serial.print("Restored ");
     Serial.print(propertyCount);
     Serial.println(" properties.");
     //*/
}


void ModeoBLE::storeCalibrations() {
    //Serial.println("Started property save");
    
    byte propertyCount = 0;
    for (byte i = 0; i < _numProperties; i++) {
        
        if (_properties[i].eepromSave && _properties[i].pendingSave) {
            int lsb = i * 2 + 1;
            int msb = i * 2;
            EEPROM.write(msb, _properties[i].value >> 8);
            EEPROM.write(lsb, _properties[i].value);
            
            propertyCount++;
        }
    }
    
    ///*
     Serial.print("Saved ");
     Serial.print(propertyCount);
     Serial.println(" properties.");
     //*/
}