#include "chickencoop.h"
#include <EEPROM.h>
#include "eeprom_anything.h"

volatile unsigned int timerS;
volatile boolean timerEnabled;
// i read somewhere that 'enum' is an 'int' -> 2 bytes
const int maxbutton = MAX_BUTTON;
volatile byte buttons[maxbutton];
const byte MAGICBYTE = 0x23;
volatile int secMs = 0;

void debug(byte* data, int length);
#define DEBUGISR(x) Serial.println(x)

// print debug message on Serial
void debug(uint8_t *data, unsigned int length) // prints 8-bit data in hex
{
  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (uint8_t i=0; i<length; i++) 
  {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39; 
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  Serial.println(tmp);
}

// constructor
ChickenCoop::ChickenCoop()
{
	loglevel = NOLOG;
 	isConfigured = false;
	requestOpenBool = false;
	requestCloseBool = false;
	timerEnabled = false;
	eventHandler = NULL;
	lightFunc = NULL;

}

// initialize configuration object with default data
void ChickenCoop::getDefaultConfig(ChickenCoopConfiguration* conf)
{
	if(loglevel>=INFO) { Serial.println("getDefaultConfig"); }
	conf->magic = MAGICBYTE;
	conf->mode = MANUAL;
	conf->switchUp = 2;
	conf->switchDown = 3;
	conf->hardwareButtons=false;
	conf->buttonUp = 0;
	conf->buttonDown = 0;
	conf->engineEnable = 4;
	conf->engineUp = 5;
	conf->engineDown = 7;
	conf->openTimeout_s = 30;
	conf->closeTimeout_s = 30;
	conf->luxThresholdDay = 120;
	conf->luxThresholdNight = 100;
	conf->errorLEDPin = 13;
}


// read configuration data from EEPROM
void  ChickenCoop::readConfig(int startaddr)
{
	if(loglevel>=INFO) { Serial.println("readConfig"); }
  EEPROM_readAnything(startaddr, configuration);
  if(configuration.magic!=MAGICBYTE) // seems to be an virgin eeprom
  {
	if(loglevel>=INFO) { Serial.println("config not read!"); }
    getDefaultConfig(&configuration);
  } 
	if(loglevel>=DEBUG) { debug((byte*)&configuration,sizeof(ChickenCoopConfiguration));}
}

// save configuration to EEPROM
void  ChickenCoop::saveConfig(int startaddr)
{
    EEPROM_writeAnything(startaddr, configuration);
}

// returns pointer to configuration object
ChickenCoopConfiguration * ChickenCoop::getConfig()
{
	return &configuration;
}

// returns pointer to system info object
const tSYSTEM * ChickenCoop::getSystemInfo() {
	return &system;
}

// configurate system with given configuration
boolean ChickenCoop::configure(ChickenCoopConfiguration config)
{
	if(loglevel>=INFO) { Serial.println("configure"); }
	// check 'magic' first ;)
	if(config.magic==MAGICBYTE)
	{
		memcpy(&configuration,&config,sizeof(config));
		isConfigured = true;
		return true;
	} else {
	if(loglevel>=INFO) { Serial.print("no valid config: "); Serial.println(config.magic); }
}
	return false;
}

// initialize system
void ChickenCoop::initSystem()
{
	system.systemstatus = SYSTEM_INIT;
	system.lightstatus = STATUSUNKNOWN;
	system.doorstatus = DOOR_UNKNOWN;
	system.errorcode = ERRORCODE_NOERROR;
	system.enginestatus = ENGINE_STOP;
	system.lux = 0;
	if(loglevel>=INFO) { Serial.println("initSystem "); debug((byte*)&system,sizeof(tSYSTEM));}
}

// init
void ChickenCoop::init() {
disableErrorTimer();
	pinMode(configuration.switchUp, INPUT); 
	digitalWrite(configuration.switchUp, HIGH); 

	pinMode(configuration.switchDown, INPUT); 
	digitalWrite(configuration.switchDown, HIGH); 

	if(configuration.hardwareButtons==true)
	{
		pinMode(configuration.buttonUp, INPUT); 
		digitalWrite(configuration.buttonUp, HIGH); 

		pinMode(configuration.buttonDown, INPUT);
		digitalWrite(configuration.buttonDown, HIGH); 
		if(loglevel>=INFO) { Serial.println("hardwareButtons enabled"); }
	}

	pinMode(configuration.engineEnable, OUTPUT);
	digitalWrite(configuration.engineEnable, LOW);

	pinMode(configuration.engineUp, OUTPUT); 
	digitalWrite(configuration.engineUp, LOW);  

	pinMode(configuration.engineDown, OUTPUT);     
	digitalWrite(configuration.engineDown, LOW);   

	pinMode(configuration.errorLEDPin, OUTPUT);           
	digitalWrite(configuration.errorLEDPin, LOW);


	for(int i = 0; i < maxbutton; i++)
	{
		buttons[i]=0xff;
	}

	initSystem();
}

// sample button
void ChickenCoop::sampleButton(tPortPIN pin, int inputpin)
{
	byte tmp = buttons[pin] << 1;
	// sample button/switch
	if(digitalRead(inputpin)==HIGH)
	{
		tmp |= 0x01;
	}
	buttons[pin] = tmp;
}

// has to be called cyclic (ISR)
void ChickenCoop::processISR(int ms) {
		secMs += ms;
	sampleButton(LIMITSWITCH_UP,configuration.switchUp);
	sampleButton(LIMITSWITCH_DOWN,configuration.switchDown);
	if(configuration.hardwareButtons==true)
	{
		sampleButton(BUTTON_UP,configuration.buttonUp);
		sampleButton(BUTTON_DOWN,configuration.buttonDown);
	}
	if(timerEnabled==true)
	{

		if((secMs>=1000)&&timerS>0)
		{
			secMs = 0;
			DEBUGISR("s");
			timerS--;
		}	
	}
}

// request open door
void ChickenCoop::requestOpen() {
	requestOpenBool = true;
	requestCloseBool = false;
}

// request close door
void ChickenCoop::requestClose() {
	requestCloseBool = true;
	requestOpenBool = false;
}

// clear requests
void ChickenCoop::clearRequest() {
	requestCloseBool = false;
	requestOpenBool = false;
}

// returns pin state (
boolean ChickenCoop::getPinStatus(tPortPIN pin) {
	byte status = buttons[pin];

	if(BUTTON_UP==pin)
	{
		if(requestOpenBool==true){status=0x00;}
	}
	if(BUTTON_DOWN==pin)
	{
		if(requestCloseBool==true){status=0x00;}
	}
// all ff (pull up) -> no press
	if((status&BUTTONMASK)==0x00){
		return true;
	}
	else {
		return false;
	}
}

// call light function
void ChickenCoop::checkLight() {
	if(lightFunc!=NULL)
	{
		float lux = lightFunc();
		if(lux >= 0 && lux < configuration.luxThresholdNight) {
			system.lightstatus = NIGHT;
		} else if (lux > configuration.luxThresholdDay) {
			system.lightstatus = DAY;
		} else if (lux < configuration.luxThresholdDay && lux > configuration.luxThresholdNight) {
			system.lightstatus = DUSKDAWN;
		}else {
			system.lightstatus = STATUSUNKNOWN;
		}
		system.lux = lux;
	}
}

// check limit switches
void ChickenCoop::checkSensors() {
	if((getPinStatus(LIMITSWITCH_UP)==true)&&(getPinStatus(LIMITSWITCH_DOWN)==false))
	{
		system.doorstatus = DOOR_OPEN;
	} else if((getPinStatus(LIMITSWITCH_UP)==false)&&(getPinStatus(LIMITSWITCH_DOWN)==true)) {
 		system.doorstatus = DOOR_CLOSE;
	} else {
		system.doorstatus = DOOR_UNKNOWN;
	}
}

//  set up/down direction
void ChickenCoop::setEngineStatus(tEngineAction action) {
	switch(action)
	{
		case ENGINE_STOP:
			if(loglevel>=INFO) { Serial.println("ENGINE_STOP"); }
			digitalWrite(configuration.engineUp,LOW);
			digitalWrite(configuration.engineDown,LOW);
			system.enginestatus = ENGINE_STOP;
		break;
		case ENGINE_MOVE_UP:
			if(loglevel>=INFO) { Serial.println("ENGINE_MOVE_UP"); }
			digitalWrite(configuration.engineUp,HIGH);
			digitalWrite(configuration.engineDown,LOW);
			system.enginestatus = ENGINE_MOVE_UP;
		break;
		case ENGINE_MOVE_DOWN:
			if(loglevel>=INFO) { Serial.println("ENGINE_MOVE_DOWN"); }
			digitalWrite(configuration.engineUp,LOW);
			digitalWrite(configuration.engineDown,HIGH);
			system.enginestatus = ENGINE_MOVE_DOWN;
		break;
		default:

		break;
	}
}

// returns door state
tDOORSTATUS ChickenCoop::getDoorStatus() {

	return system.doorstatus;
}

// turn engine on
void ChickenCoop::enableEngine() {
	if(loglevel>=INFO) { Serial.println("enableEngine"); }
	digitalWrite(configuration.engineEnable,HIGH);
}

// turn engine off
void ChickenCoop::disableEngine() {
if(loglevel>=INFO) { Serial.println("disableEngine"); }
	digitalWrite(configuration.engineEnable,LOW);
}

// configure & enable error timer with given interval
void ChickenCoop::enableErrorTimer(unsigned int intval) {
	if(loglevel>=INFO) { Serial.print("enableErrorTimer "); Serial.println(intval); }
	secMs = 0;
	timerS = intval;
	timerEnabled = true;
}

// disable error timer
void ChickenCoop::disableErrorTimer() {
	if(loglevel>=INFO) { Serial.println("disableErrorTimer"); }
	timerEnabled = false;
}

// returns light state (DAY,NIGHT,UNKNOWN)
tLightStatus ChickenCoop::getLightStatus() {
	return system.lightstatus;
}

// returns true if error timeout has occured
boolean ChickenCoop::errorTimeout() {
	boolean timeout = false;
	if(timerEnabled==true&&timerS==0)
	{
		timeout = true;
	}
	return timeout;
}

// flash led
void ChickenCoop::flashLED()
{
	static unsigned long time = millis() + 1500;
	if(millis()>=time)
	{
		time = millis() + 1500;
		digitalWrite(ERRORLED, !digitalRead(ERRORLED));
	}
	
}

// set light function pointer
void ChickenCoop::setLightFunc(chickencoop_lightfunc * lightfunction)
{
	lightFunc = lightfunction;
}

// set event handler function pointer
void ChickenCoop::setEventHandler(chickencoop_eventHandler * handler)
{
	eventHandler = handler;
}

// calles event handler function
void ChickenCoop::updateSystem()
{
	if(loglevel>=INFO) { Serial.println("updateSystem"); }
	if(eventHandler!=NULL)
	{
		eventHandler(&system);
	}
}

// set system state
void ChickenCoop::setSystemState(tSYSTEMSTATUS s)
{
	system.systemstatus = s;
//	debug("state -> %i",(int)s);
	updateSystem();
}

// main function
// all the magic happens here ;)
// has to be called in loop function
void ChickenCoop::process() {
	switch(system.systemstatus) {
		case  SYSTEM_INIT:
			checkSensors();
			checkLight();
			if(getDoorStatus()==DOOR_CLOSE) {
				setSystemState(SYSTEM_CLOSE);
			} else if(getDoorStatus()==DOOR_OPEN) {
				setSystemState(SYSTEM_OPEN);
			} else {
				if(getPinStatus(BUTTON_UP)==true) {
					setSystemState(SYSTEM_START_OPENING);
				}
				if(getPinStatus(BUTTON_DOWN)==true) {
					setSystemState(SYSTEM_START_CLOSING);
				}
			}
		break;
 		case  SYSTEM_START_OPENING:
			checkSensors();
			setEngineStatus(ENGINE_MOVE_UP);
			enableEngine();
			setSystemState(SYSTEM_OPENING);
			enableErrorTimer(configuration.openTimeout_s);
		break;
 		case  SYSTEM_START_CLOSING:
			checkSensors();
			setEngineStatus(ENGINE_MOVE_DOWN);
			enableEngine();
			setSystemState(SYSTEM_CLOSING);
			enableErrorTimer(configuration.closeTimeout_s);
		break;
 		case  SYSTEM_CLOSING:
			checkSensors();
			// check for errortimer
			if(errorTimeout()==true)
			{
				setSystemState(SYSTEM_START_ERROR);
			}
			if(getDoorStatus()==DOOR_CLOSE) {
				disableEngine();
				setEngineStatus(ENGINE_STOP);
				disableErrorTimer();
				setSystemState(SYSTEM_CLOSE);
			}
			if(getPinStatus(BUTTON_UP)==true)
			{
				setSystemState(SYSTEM_START_OPENING);
			}
		break;
		case  SYSTEM_OPENING:
			checkSensors();
			// check for errortimer
			if(errorTimeout()==true)
			{
				setSystemState(SYSTEM_START_ERROR);
			}
			if(getDoorStatus()==DOOR_OPEN) {
				disableEngine();
				setEngineStatus(ENGINE_STOP);
				disableErrorTimer();
				setSystemState(SYSTEM_OPEN);
			}
			if(getPinStatus(BUTTON_DOWN)==true)
			{
				setSystemState(SYSTEM_START_CLOSING);
			}
		break;
 		case  SYSTEM_OPEN:
			// check tmr1: do adc
			checkLight();
			checkSensors();
			if(getPinStatus(BUTTON_DOWN)==true|| 
				(configuration.mode==AUTO && getLightStatus()==NIGHT )) {
				setSystemState(SYSTEM_START_CLOSING);
			} else {
				// goto sleep
//				goSleep();
			}
		break;
 		case  SYSTEM_CLOSE:
			// check tmr1: do adc
			checkLight();
			checkSensors();
			if(getPinStatus(BUTTON_UP)==true ||
				(configuration.mode==AUTO && getLightStatus()==DAY)) {
				setSystemState(SYSTEM_START_OPENING);
			} else {
				// goto sleep
//				goSleep();
			}
		break;
 		case SYSTEM_START_ERROR:
				setEngineStatus(ENGINE_STOP);
				disableErrorTimer();
				disableEngine();
				setSystemState(SYSTEM_ERROR);
		break; 		
		case SYSTEM_ERROR:
				// flash error led
			flashLED();
		break;
		default:

		break;
	} 
	clearRequest();
}


