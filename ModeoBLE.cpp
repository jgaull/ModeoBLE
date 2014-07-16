/*
 ModeoBLE Library
 Jon Gaull
 Copywrite 2013
 */

#define NUM_SENSORS 8 //Is there a better way to do this?

//Request Identifiers
#define REQUEST_CONNECT 10
#define REQUEST_DISCONNECT 11
#define REQUEST_GET_PROPERTY_VALUE 12
#define REQUEST_GET_SENSOR_VALUE 15
#define REQUEST_WRITE_GET_PROPERTY 18
#define REQUEST_SET_PROPERTY 19
#define REQUEST_WRITE_PROPERTY 20
#define REQUEST_SET_CONFIG 21
#define REQUEST_WRITE_CONFIG 22

#define STATE_OFF 0
#define STATE_ON 1

#define FIRST_CONFIG_BYTE 128
#define CONFIG_PROPERTY 1
#define CONFIG_SENSOR 2

#include <AltSoftSerial.h>
#include <EEPROM.h>

#include "Arduino.h"
#include "ModeoBLE.h"

struct Property {
    //unsigned short value;
    byte valueSize;
    byte valueIndex;
    byte identifier;
    bool pendingSave;
    bool eepromSave;
    bool callbackOnChange;
};

struct Sensor {
    byte identifier;
    unsigned short value;
};

//Remember to change this when you add properties!!! I wish I knew more about C++...
byte _values[51];
byte _valuesLength = 0;

//And this!!! Temporariliy...
Property _properties[9];
byte _propertiesLength = 0;
byte _numProperties = 0;
Property _propertyPendingSave;
byte _propertyIdentifierForPropertyPendingSave;

Sensor _sensors[9];
byte _sensorsLength = 0;
byte _numSensors = 0;

byte _versionNumber = 0;

byte _previousWriteRequest[20];
byte _previousWriteRequestLength = 0;

AltSoftSerial _bleMini;
Callback *_callback;

byte _availableBytes = 0;
unsigned int _lastByteReceivedTimestamp = 0;

byte _state = STATE_OFF;

ModeoBLE::ModeoBLE() {
    Serial.println("ModeoBLE Constructed.");
}

void ModeoBLE::startup(Callback *callback) {
    
    _callback = callback;
    _bleMini.begin(57600);
    
    loadConfiguration();
    //Serial.println("Configuration loaded");
    
    if (_state == STATE_OFF && _numProperties == _propertiesLength && _numSensors == _sensorsLength) {
        //retrieveCalibrations();
        _state = STATE_ON;
        
        Serial.println("ModeoBLE Startup!");
    }
    else {
        //Serial.println("Failure at startup.");
        
        if (_numProperties != _propertiesLength) {
            Serial.println("_numProperties does not match _propertiesLength.");
        }
        
        if (_numSensors != _sensorsLength) {
            Serial.print("_numSensors = ");
            Serial.println(_numSensors);
            Serial.print("_sensorsLength = ");
            Serial.println(_sensorsLength);
        }
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

void ModeoBLE::getValueForProperty(byte identifier, byte *length, byte data[]) {
    //Serial.println("Get Value");
    
    int index = indexForProperty(identifier);
    if (index != -1) {
        *length = _properties[index].valueSize;
        byte firstIndex = _properties[index].valueIndex;
        
        for (int i = 0; i < *(length); i++) {
            data[i] = _values[i + firstIndex];
        }
    }
}

void ModeoBLE::setValueForProperty(byte identifier, byte data[]) {
    //Serial.println("Set Value");
    
    int index = indexForProperty(identifier);
    if (index != -1) {
        byte length = _properties[index].valueSize;
        byte firstIndex = _properties[index].valueIndex;
        
        for (byte i = 0; i < length; i++) {
            if (_values[i + firstIndex] != data[i]) {
                _values[i + firstIndex] = data[i];
                _properties[index].pendingSave = true;
            }
        }
    }
}

void ModeoBLE::setUnsignedShortValueForProperty(unsigned short value, byte identifier) {
    byte data[2];
    data[0] = value;
    data[1] = value >> 8;
    setValueForProperty(identifier, data);
}

unsigned short ModeoBLE::getUnsignedShortValueForProperty(byte identifier) {
    //It sucks to have to call indexForProperty here.
    int index = indexForProperty(identifier);
    if (index != -1) {
        //I'm only calling it because I need the length to allocate an array to hold the result.
        //I'm guessing that this could be done better.
        byte length = _properties[index].valueSize;
        byte data[length];
        
        getValueForProperty(identifier, &length, data);
        
        unsigned short value = (data[1] << 8) + data[0];
        return value;
    }
}

byte ModeoBLE::getByteValueForProperty(byte identifier) {
    int index = indexForProperty(identifier);
    if (index != -1) {
        byte length = _properties[index].valueSize;
        byte data[length];
        getValueForProperty(identifier, &length, data);
        return data[0];
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

/*
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
 */

//Debug funtions
/*
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
}*/

/*
void ModeoBLE::clearEEPROM() {
    Serial.println("Clear EEPROM not hooked up.");
}*/

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

/*
int ModeoBLE::indexForBezier(byte identifier) {
    for (byte i = 0; i < _beziersLength; i++) {
        if (_beziers[i].identifier == identifier) {
            return i;
        }
    }
    
    return -1;
}
 */


//BLE Shit
void ModeoBLE::performBluetoothReceive() {
    
    byte currentlyAvailable = _bleMini.available();
    if (currentlyAvailable != _availableBytes) {
        _lastByteReceivedTimestamp = millis();
        _availableBytes = currentlyAvailable;
    }
    
    //Serial.print("currentlyAvailable = ");
    //Serial.println(currentlyAvailable);
    
    //this shit is all kinds of rolling over.
    int timePassed = millis() - _lastByteReceivedTimestamp;
    
    if ( _availableBytes > 0 && timePassed > 50 ) {
        
        //Serial.print("currentlyAvailable = ");
        //Serial.println(_availableBytes);
        
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
                
            case REQUEST_GET_SENSOR_VALUE:
                Serial.println("get sensor value");
                getSensorValue();
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
                
            case REQUEST_SET_CONFIG:
                Serial.println("set config");
                setConfig();
                break;
                
            case REQUEST_WRITE_CONFIG:
                Serial.println("write config");
                writeConfig();
                break;
                
            default:
                Serial.print("Unknown command: ");
                Serial.println(identifier);
        }
        
        clearBLEBuffer();
        _availableBytes = 0;
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
        
        int index = indexForProperty(propertyIdentifier);
        /*
        Serial.print("propertyIdentifier = ");
        Serial.println(propertyIdentifier);
        
        Serial.print("index = ");
        Serial.println(index);
        //*/
        if ( index != -1) {
            byte valueIndex = _properties[index].valueIndex;
            byte valueSize = _properties[index].valueSize;
            /*
            Serial.print("valueIndex = ");
            Serial.println(valueIndex);
            
            Serial.print("valueSize = ");
            Serial.println(valueSize);
            //*/
            
            _bleMini.write(REQUEST_GET_PROPERTY_VALUE);
            _bleMini.write(propertyIdentifier);
            _bleMini.write(valueSize);
            
            /*
            Serial.print("REQUEST_GET_PROPERTY_VALUE = ");
            Serial.println(REQUEST_GET_PROPERTY_VALUE);
            
            Serial.print("propertyIdentifier = ");
            Serial.println(propertyIdentifier);
            
            Serial.print("valueSize = ");
            Serial.println(valueSize);
             //*/
            
            for (byte i = 0; i < valueSize; i++) {
                _bleMini.write(_values[i + valueIndex]);
            }
        }
        else {
            Serial.println("oops 2");
            clearBLEBuffer();
        }
    }
    else {
        Serial.println("oops 1");
        clearBLEBuffer();
    }
}

void ModeoBLE::writeGetProperty() {
    if ( _bleMini.available() >= 1 ) {
        byte propertyIdentifier = _bleMini.read();
        
        int index = indexForProperty(propertyIdentifier);
        byte valueIndex = _properties[index].valueIndex;
        byte size = _properties[index].valueSize;
        
        /*
        Serial.print("propertyIdentifier = ");
        Serial.println(propertyIdentifier);
        
        Serial.print("index = ");
        Serial.println(index);
        
        Serial.print("valueIndex = ");
        Serial.println(valueIndex);
        
        Serial.print("size = ");
        Serial.println(size);
        
        Serial.print("_bleMini.available() = ");
        Serial.println(_bleMini.available());
        //*/
        
        if (_bleMini.available() >= size) {
            bool valid = true;
            for (int i = 0; i < size; i++) {
                byte bleValue = _bleMini.read();
                byte value = _values[valueIndex + i];
                
                if (bleValue != _values[valueIndex + i]) {
                    valid = false;
                    break;
                }
            }
            
            if (valid) {
                Serial.println("valid!");
                _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
                _bleMini.write(1);
            }
            else {
                Serial.println("invalid!");
                _bleMini.write(REQUEST_WRITE_GET_PROPERTY);
                _bleMini.write((byte)0);
            }
        }
        else {
            Serial.println("oops 2");
            clearBLEBuffer();
        }
    }
    else {
        Serial.println("oops 1");
        clearBLEBuffer();
    }
}

void ModeoBLE::setProperty() {
    
    if ( _bleMini.available() >= 1) {
        byte headerSize = 2;
        byte propertyIdentifier = _bleMini.read();
        int index = indexForProperty(propertyIdentifier);
        
        byte numBytes = _properties[index].valueSize;
        _previousWriteRequestLength = headerSize + numBytes;
        
        /*
        Serial.print("propertyIdentifier = ");
        Serial.println(propertyIdentifier);
        
        Serial.print("index = ");
        Serial.println(index);
         */
        
        if (_bleMini.available() >= numBytes) {
            
            _previousWriteRequest[0] = REQUEST_SET_PROPERTY;
            _previousWriteRequest[1] = propertyIdentifier;
            for (byte i = headerSize; i < _previousWriteRequestLength; i++) {
                _previousWriteRequest[i] = _bleMini.read();
            }
            
            for (byte i = 0; i < _previousWriteRequestLength; i++) {
                _bleMini.write(_previousWriteRequest[i]);
            }
        }
        else {
            Serial.println("oops 1");
            clearBLEBuffer();
        }
    }
    else {
        Serial.println("oops 2");
        clearBLEBuffer();
    }
}

void ModeoBLE::writeProperty() {
    byte headerSize = 2;
    byte propertyIdentifier = _previousWriteRequest[1];
    
    int index = indexForProperty(propertyIdentifier);
    byte valueIndex = _properties[index].valueIndex;
    byte valueSize = _properties[index].valueSize;
    
    //boolean didChange = false;
    
    for (byte i = 0; i < valueSize; i++) {
        /*
        if (_values[i + valueIndex] != _previousWriteRequest[i + headerSize]) {
            didChange = true;
        }
         */
        
        _values[i + valueIndex] = _previousWriteRequest[i + headerSize];
    }
    
    _properties[index].pendingSave = true;
    
    if (_properties[index].callbackOnChange) {
        byte value[valueSize];
        
        for (byte i = 0; i < valueSize; i++) {
            value[i] = _values[i + valueIndex];
        }
        
        (*_callback)(propertyIdentifier, valueSize, value);
    }
    
    _bleMini.write(REQUEST_WRITE_PROPERTY);
    _bleMini.write(true);
    
    /*
    Serial.print("unsignedShortValue = ");
    Serial.println(getUnsignedShortValueForProperty(propertyIdentifier));
     */
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

void ModeoBLE::setConfig() {
    if ( _bleMini.available() >= 1) {
        byte headerSize = 2;
        byte offset = _bleMini.read();
        
        _previousWriteRequest[0] = REQUEST_SET_CONFIG;
        _previousWriteRequest[1] = offset;
        
        _previousWriteRequestLength = headerSize;
        while (_bleMini.available()) {
            _previousWriteRequest[_previousWriteRequestLength] = _bleMini.read();
            _previousWriteRequestLength++;
        }
        
        Serial.print("_previousWriteRequestLength = ");
        Serial.println(_previousWriteRequestLength);
        
        for (byte i = 0; i < _previousWriteRequestLength; i++) {
            _bleMini.write(_previousWriteRequest[i]);
        }
    }
    else {
        Serial.println("oops 2");
        clearBLEBuffer();
    }
}

void ModeoBLE::writeConfig() {
    byte headerSize = 2;
    byte offset = _previousWriteRequest[1];
    
    for (byte i = 0; i < _previousWriteRequestLength; i++) {
        EEPROM.write(FIRST_CONFIG_BYTE + offset + i, _previousWriteRequest[i + headerSize]);
    }
    
    _bleMini.write(REQUEST_WRITE_CONFIG);
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
            
            byte valueSize = _properties[i].valueSize;
            byte valueIndex = _properties[i].valueIndex;
            
            for (byte j = 0; j < valueSize; j++) {
                byte value = EEPROM.read(valueIndex + j);
                _values[valueIndex + j] = value;
            }
            
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
        
        /*
        Serial.print("_properties[");
        Serial.print(i);
        Serial.print("].eepromSave = ");
        Serial.println(_properties[i].eepromSave);
        
        Serial.print("_properties[");
        Serial.print(i);
        Serial.print("].pendingSave = ");
        Serial.println(_properties[i].pendingSave);
         */
        
        if (_properties[i].eepromSave && _properties[i].pendingSave) {
            
            byte valueSize = _properties[i].valueSize;
            byte valueIndex = _properties[i].valueIndex;
            
            for (byte j = 0; j < valueSize; j++) {
                EEPROM.write(valueIndex + j, _values[valueIndex + j]);
            }
            
            propertyCount++;
            
            _properties[i].pendingSave = false;
        }
    }
    
    ///*
    Serial.print("Saved ");
    Serial.print(propertyCount);
    Serial.println(" properties.");
    //*/
}

void ModeoBLE::loadConfiguration() {
    
    //Serial.println("Loading configuration!");
    /*
    for (byte i = 0; i < 50; i++) {
        byte value = EEPROM.read(i + FIRST_CONFIG_BYTE);
        Serial.print("value[");
        Serial.print(i + FIRST_CONFIG_BYTE);
        Serial.print("] = ");
        Serial.println(value);
    }
     */
    
    byte headerSize = 2;
    //read the configuration header data
    _numProperties = EEPROM.read(FIRST_CONFIG_BYTE + 0);
    _numSensors = EEPROM.read(FIRST_CONFIG_BYTE + 1);
    
    /*
    Serial.print("_numProperties = ");
    Serial.println(_numProperties);
    Serial.print("_numSensors = ");
    Serial.println(_numSensors);
     //*/
    
    //setup some variables
    byte parsedProperties = 0;
    byte parsedSensors = 0;
    int currentIndex = headerSize + FIRST_CONFIG_BYTE;
    
    //iterate over EEPROM storage until we've loaded everything we need.
    while ((parsedProperties < _numProperties || parsedSensors < _numSensors) && currentIndex < FIRST_CONFIG_BYTE + 256) {
        
        //read the configuration data header.
        byte configType = EEPROM.read(currentIndex);
        byte identifier = EEPROM.read(currentIndex + 1);
        currentIndex += 2;
        
        /*
        Serial.print("configType = ");
        Serial.println(configType);
        Serial.print("identifier = ");
        Serial.println(identifier);
         */
         
        //if we have a property
        if (configType == CONFIG_PROPERTY) {
            //read the property data
            byte size = EEPROM.read(currentIndex + 0);
            byte callbackOnChange = EEPROM.read(currentIndex + 1);
            byte eepromSave = EEPROM.read(currentIndex + 2);
            byte permissions = EEPROM.read(currentIndex + 3);
            byte dataType = EEPROM.read(currentIndex + 4);
            currentIndex += 5;
            
            byte value[size];
            for (byte i = 0; i < size; i++) {
                value[i] = EEPROM.read(currentIndex + i);
            }
            
            currentIndex += size;
            
            //and register the property
            registerProperty(identifier, size, eepromSave, callbackOnChange, value);
            parsedProperties++;
        }
        //if we have a sensor
        else if (configType == CONFIG_SENSOR) {
            //There's no more data to read (yet). Register the sensor.
            registerSensor(identifier);
            parsedSensors++;
        }
        
        /*
        Serial.print("currentIndex = ");
        Serial.println(currentIndex);
         */
    }
}

void ModeoBLE::registerProperty(byte identifier, byte size, bool eepromSave, bool callbackOnChange, byte value[]) {
    
    if (indexForProperty(identifier) == -1) {
        _properties[_propertiesLength].identifier = identifier;
        _properties[_propertiesLength].valueSize = size;
        _properties[_propertiesLength].valueIndex = _valuesLength;
        _properties[_propertiesLength].pendingSave = false;
        _properties[_propertiesLength].eepromSave = eepromSave;
        _properties[_propertiesLength].callbackOnChange = callbackOnChange;
        
        for (byte i = 0; i < size; i++) {
            byte datem = value[i];
            _values[_valuesLength + i] = datem;
        }
        
        _propertiesLength++;
        _valuesLength += size;
        
        //Serial.println("Property Registered.");
        //Serial.print("_valuesLength = ");
        //Serial.println(_valuesLength);
    }
    else {
        Serial.print("Property with identifier ");
        Serial.print(identifier);
        Serial.println(" already registered.");
    }
}

/*
void ModeoBLE::registerPropertyWithCallback(byte identifier, byte size, bool eepromSave, Callback *callback) {
    
    if (indexForProperty(identifier) == -1) {
        _properties[_propertiesLength].identifier = identifier;
        _properties[_propertiesLength].valueSize = size;
        _properties[_propertiesLength].valueIndex = _valuesLength;
        _properties[_propertiesLength].pendingSave = false;
        _properties[_propertiesLength].eepromSave = eepromSave;
        _properties[_propertiesLength].callbackOnChange = true;
        _properties[_propertiesLength].callback = callback;
        
        _propertiesLength++;
        _valuesLength += size;
        
        Serial.println("Property Registered.");
        //Serial.print("_valuesLength = ");
        //Serial.println(_valuesLength);
    }
    else {
        Serial.println("Property has already been registered");
    }
}
*/

void ModeoBLE::registerSensor(byte identifier) {
    if (indexForSensor(identifier) == -1) {
        _sensors[_sensorsLength].identifier = identifier;
        _sensors[_sensorsLength].value = 0;
        _sensorsLength++;
        
        /*
        Serial.print("Sensor with identifier ");
        Serial.print(identifier);
        Serial.println(" registered.");
         //*/
    }
    else {
        Serial.print("Sensor with identifier ");
        Serial.print(identifier);
        Serial.println(" already registered.");
    }
}





