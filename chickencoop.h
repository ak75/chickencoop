
#ifndef chickencoop_h
#define chickencoop_h

#define ERRORLED 13

//make it a little prettier on the front end. 
#define details(name) (byte*)&name,sizeof(name)

//Not neccessary, but just in case. 
#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Stream.h"
#include <stdio.h>
#include <stdint.h>
#include <Wire.h>

#define BUTTONMASK 0x0f

typedef enum eCoopLog {
	NOLOG,
	INFO,
	DEBUG,
	TRACE
} tCoopLog;

typedef enum eCoopMode {
	AUTO,
	MANUAL
 } tCoopMode;

typedef struct ChickenCoopConfiguration {

	tCoopMode mode;
	int switchUp;
	int switchDown;
	int engineEnable;
	int engineUp;
	int engineDown;
	int buttonUp;
	int buttonDown;
	boolean hardwareButtons;
	int openTimeout_s;
	int closeTimeout_s;
	int luxThresholdDay;
	int luxThresholdNight;
	int errorLEDPin;
	byte magic;
} ChickenCoopConfiguration;


typedef enum eLightStatus {
STATUSUNKNOWN,
DAY,
DUSKDAWN,
NIGHT } tLightStatus;

typedef enum eSYSTEMSTATUS{
 SYSTEM_INIT,
 SYSTEM_START_OPENING, 
 SYSTEM_OPENING, 
 SYSTEM_OPEN, 
 SYSTEM_START_CLOSING, 
 SYSTEM_CLOSING, 
 SYSTEM_CLOSE, 
 SYSTEM_START_ERROR,
 SYSTEM_ERROR
  } tSYSTEMSTATUS;

typedef enum eDOORSTATUS{
 DOOR_UNKNOWN,
 DOOR_OPEN, 
 DOOR_CLOSE } tDOORSTATUS;


typedef enum eEngineAction{
 ENGINE_STOP,
 ENGINE_MOVE_UP,
 ENGINE_MOVE_DOWN, 
 } tEngineAction;

 typedef enum ePortPin {
 BUTTON_UP,
 BUTTON_DOWN,
 LIMITSWITCH_UP,
 LIMITSWITCH_DOWN,
 MAX_BUTTON
 } tPortPIN;

 typedef enum eErrorCode {
 ERRORCODE_NOERROR,
 ERRORCODE_UNKNOWN,
 ERRORCODE_DOOR,
 ERRORCODE_SENSOR,
 ERRORCODE_ENGINE,
 ERRORCODE_LIGHT
 } tErrorCode;


typedef struct sSYSTEM {
tSYSTEMSTATUS systemstatus;
tLightStatus lightstatus;
tDOORSTATUS doorstatus;
tErrorCode errorcode;
tEngineAction enginestatus;
float lux;
} tSYSTEM;

// Event handler
typedef void(chickencoop_eventHandler)(const tSYSTEM * sysdata);
typedef float(chickencoop_lightfunc)(void);

class ChickenCoop {
public:
tCoopLog loglevel;
ChickenCoop();
void getDefaultConfig(ChickenCoopConfiguration*);
boolean configure(ChickenCoopConfiguration);
ChickenCoopConfiguration * getConfig();
void readConfig(int startaddr);
void saveConfig(int startaddr);
void init();
void processISR(int ms);
void process();

void requestOpen();
void requestClose();

void setEventHandler(chickencoop_eventHandler * eventHandler);
void setLightFunc(chickencoop_lightfunc * lightfunc);
const tSYSTEM * getSystemInfo();

private:
void initSystem();
void setSystemState(tSYSTEMSTATUS s);
void sampleButton(tPortPIN pin, int inputpin);
void clearRequest();
void enableEngine();
void disableEngine();
void setEngineStatus(tEngineAction action);
void updateSystem();
void checkSensors();
tLightStatus getLightStatus();
void checkLight();
void enableErrorTimer(unsigned int intval);
void disableErrorTimer();
void flashLED();
boolean errorTimeout();
tDOORSTATUS getDoorStatus();
boolean getPinStatus(tPortPIN pin);
boolean isConfigured;
boolean requestOpenBool;
boolean requestCloseBool;
ChickenCoopConfiguration configuration;
tSYSTEM system;

chickencoop_eventHandler * eventHandler;
chickencoop_lightfunc * lightFunc;

};



#endif
