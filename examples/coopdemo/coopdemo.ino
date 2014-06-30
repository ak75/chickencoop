#include "Arduino.h"
#include <SPI.h>

#include <SerialCommand.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

#include <chickencoop.h>
#include <TimerOne.h>                           // Header file for TimerOne library

#define TIMER_US 20000                         // 20mS set timer duration in microseconds 
const int SENSOR_INT = 20;
int STARTADDR = 0;

ChickenCoop coop;
ChickenCoopConfiguration coopconfig;
const tSYSTEM* systeminfo;
// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

#define Communication Serial1

SerialCommand SCmd;   // The demo SerialCommand object

void timerIsr()
{
  coop.processISR(SENSOR_INT);
}

void configureTSL()
{
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  tsl.begin();
}

float getLight()
{
/* Get a new sensor event */ 
  sensors_event_t event;
configureTSL();
  tsl.getEvent(&event);
  return event.light;
}

void systemUpdate(const tSYSTEM* system)
{
  Communication.print("update:");
  byte* p = (byte*)system;
  PrintHex83(p, sizeof(tSYSTEM));
  //  for (int i = 0 ; i < sizeof(tSYSTEM) ; i++)
  //  {
  //    Communication.print((unsigned char) * (p + i), HEX);
  //  }
  //
  Communication.println();
}

void reset() {
  coop.init();
}

byte hex2bin( const char *s )
{
  char buff[3];
  buff[0] = *s;
  buff[1] = *(s+1);
    buff[2] = '\0';

  unsigned long ret = strtoul(buff,NULL,16);
  return (byte)ret;
}

void setConfiguration()
{
  const byte buffsize = (sizeof(ChickenCoopConfiguration));
  ChickenCoopConfiguration newConf;
  byte* buffindex = (byte*)&newConf;
  char *arg;

  arg = SCmd.next();
  if (strcmp("raw", arg) == 0)
  {
    arg = SCmd.next();
    for (int i = 0; i < buffsize ; i++)
    {
      *(buffindex++) = hex2bin((arg + (i*2)));
    }
    if (coop.configure(newConf) == true)
    {
      Communication.println("setconfig:OK");
    } else {
      Communication.println("setconfig:FAIL");
    }
  } else if (strcmp("lux", arg) == 0)
  {
    arg = SCmd.next();
    if (strcmp("night", arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->luxThresholdNight = atoi(arg);
    } else if (strcmp("day", arg) == 0) {
      arg = SCmd.next();
      coop.getConfig()->luxThresholdDay = atoi(arg);
    } else {}
  } else if (strcmp("timeout", arg) == 0)
  {
    arg = SCmd.next();
    if (strcmp("open", arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->openTimeout_s = atoi(arg);
    } else if (strcmp("close", arg) == 0) {
      arg = SCmd.next();
      coop.getConfig()->closeTimeout_s = atoi(arg);
    } else {}
  }
  else if (strcmp("mode", arg) == 0)
  {
    arg = SCmd.next();
    coop.getConfig()->mode = (tCoopMode)atoi(arg);
  } else if (strcmp("button", arg) == 0)
  {
    arg = SCmd.next();
    if (strcmp("enabled", arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->hardwareButtons = (boolean)atoi(arg);
    } else if (strcmp("up", arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->buttonUp = atoi(arg);
    } else if (strcmp("down", arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->buttonDown = atoi(arg);
    } else {}
  } else if (strcmp("engine", arg) == 0)
  {
    arg = SCmd.next();
    if (strcmp("enable" , arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->engineEnable = atoi(arg);
    } else if (strcmp("up" , arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->engineUp = atoi(arg);
    } else if (strcmp("down" , arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->engineDown = atoi(arg);
    } else {}
  } else if (strcmp("switch", arg) == 0)
  {
    arg = SCmd.next();
    if (strcmp("up" , arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->switchUp = atoi(arg);
    } else if (strcmp("down" , arg) == 0)
    {
      arg = SCmd.next();
      coop.getConfig()->switchDown = atoi(arg);
    } else {}
  } else if (strcmp("magic", arg) == 0)
  {
    arg = SCmd.next();
    coop.getConfig()->magic = (byte)atoi(arg);
  } else {
    Communication.print("???");
  }

}

void getConfiguration()
{
  char *arg = SCmd.next();
  if (arg != NULL)
  {
    if (strcmp("lux", arg) == 0)
    {
      Communication.print("night:");
      Communication.println(coop.getConfig()->luxThresholdNight);
      Communication.print("day:");
      Communication.println(coop.getConfig()->luxThresholdDay);
    } else if (strcmp("timeout", arg) == 0)
    {
      Communication.print("open:");
      Communication.println(coop.getConfig()->openTimeout_s);
      Communication.print("close:");
      Communication.println(coop.getConfig()->closeTimeout_s);
    }

    else if (strcmp("mode", arg) == 0)
    {
      Communication.print("mode:");
      Communication.println(coop.getConfig()->mode);
    } else if (strcmp("button", arg) == 0)
    {
      Communication.print("enabled:");
      Communication.println(coop.getConfig()->hardwareButtons);
      Communication.print("up:");
      Communication.println(coop.getConfig()->buttonUp);
      Communication.print("down:");
      Communication.println(coop.getConfig()->buttonDown);
    } else if (strcmp("engine", arg) == 0)
    {
      Communication.print("enable:");
      Communication.println(coop.getConfig()->engineEnable);
      Communication.print("up:");
      Communication.println(coop.getConfig()->engineUp);
      Communication.print("down:");
      Communication.println(coop.getConfig()->engineDown);
    } else if (strcmp("switch", arg) == 0)
    {
      Communication.print("up:");
      Communication.println(coop.getConfig()->switchUp);
      Communication.print("down:");
      Communication.println(coop.getConfig()->switchDown);
    }
  } else {
    Communication.print("config:");
    byte* p = (byte*)coop.getConfig();
    PrintHex83(p, sizeof(ChickenCoopConfiguration));
    Communication.println();
  }
}

void saveConfiguration()
{
  coop.saveConfig(STARTADDR);
  Communication.println("saveconfig:OK");
}

void set()
{
  char *arg = SCmd.next();
  if (arg != NULL)
  {
    if (strcmp("open", arg) == 0)
    {
      coop.requestOpen();
    }
    if (strcmp("close", arg) == 0)
    {
      coop.requestClose();
    }
  }
}

void request()
{
  char *arg = SCmd.next();
  if (arg != NULL)
  {
    if (strcmp("lightraw", arg) == 0)
    {
      Communication.print("lux:");
      Communication.print(systeminfo->lux);
    }
    if (strcmp("light", arg) == 0)
    {
      Communication.print("light:");
      Communication.print(systeminfo->lightstatus);
    }
    if (strcmp("door", arg) == 0)
    {
      Communication.print("door:");
      Communication.print(systeminfo->doorstatus);
    }

    if (strcmp("system", arg) == 0)
    {
      Communication.print("system:");
      Communication.print(systeminfo->systemstatus, HEX);
    }

    if (strcmp("error", arg) == 0)
    {
      Communication.print("error:");
      Communication.print(systeminfo->errorcode);
    }
    if (strcmp("engine", arg) == 0)
    {
      Communication.print("engine:");
      Communication.print(systeminfo->enginestatus);
    }
    if (strcmp("update", arg) == 0)
    {
      systemUpdate(systeminfo);
    }
    Communication.println();
  }
}

void unrecognized()
{
  Communication.println("wtf!?");
  Communication.println("getConfig");
  Communication.println("setConfig");
  Communication.println("saveConfig");
  Communication.println("get");
  Communication.println("set");
  Communication.println("reset");
}

void setup() {

  Serial.begin(57600);
  // while the serial stream is not open, do nothing:
//  while (!Serial) ;
  Serial.println("hello world");
  // init bt connection
  Serial1.begin(9600);
  // configure serial commands
  SCmd.addCommand("getconfig", getConfiguration);
  SCmd.addCommand("setconfig", setConfiguration);
  SCmd.addCommand("saveconfig", saveConfiguration);
  SCmd.addCommand("get", request);
  SCmd.addCommand("set", set);
  SCmd.addCommand("reset", reset);
  SCmd.addDefaultHandler(unrecognized);

	configureTSL();

  coop.loglevel = DEBUG;
  coop.readConfig(STARTADDR);
  //  ChickenCoopConfiguration* conf = coop.getConfig();
  // conf->closeTimeout_s = 15;
  // initialize system
  coop.init();
  // set event handler for notifys
  coop.setEventHandler( systemUpdate );
  // set function for lux data retrieval
  coop.setLightFunc( getLight );
  // get systeminfo struct
  systeminfo = coop.getSystemInfo();
  // initialize timer (for 10ms interrupts)
  Timer1.initialize(TIMER_US);                  // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );           // attach the ISR routine here

}

void loop() {

  SCmd.readSerial();
  coop.process();

}

void PrintHex83(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length * 2 + 1];
  byte first ;
  int j = 0;
  for (uint8_t i = 0; i < length; i++)
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
  tmp[length * 2] = 0;

  Communication.println(tmp);
}

