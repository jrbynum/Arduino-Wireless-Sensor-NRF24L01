/*
 Copyright (C) 2014 Jonathan R. Bynum <jrbynum@shaw.ca>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

//***************************************
// Wireless Sensor Node  - Uses
// RTC DS3231
// Temp Sensor DS18B20
// Radio - NRF24L01
//***************************************

//*********************************************************
// Uses a ATmega328 running at 8Mhz on my own special board 
//*********************************************************  

#include <Wire.h>
#include <config.h>
#include <ds3231.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <MsTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Narcoleptic.h>

//define this to print debug messages to the serial port
//#define DEBUG 1

int sleepDelay = 10000;	// in milliseconds

// DS18B20 is plugged into Arduino D4 
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// This is DS18B20 address - run address finder to find the address of the temp sensor and place it here
DeviceAddress Therm1 = { 0x28, 0xEA, 0x78, 0x8F, 0x05, 0x00, 0x00, 0xDD };
//float temp1F;


#define BUFF_MAX 20
//**********************************************************************
// some string buffers to send information via radio
//**********************************************************************

uint8_t time[8];
char tbuff[BUFF_MAX];  //string buffer to hold the date and time
char stemperature[7]; //string buffer to hold the temperature in celsius
char svoltage[7];     //string buffer to hold volatge
char swindspeed[7];   //string buffer to hold windspeed in KMPH
//***********************************************************************
// The measurement variables
//***********************************************************************

float temp1C;
float Vcc; 	//Supplied Voltage
float wspeedkpm;

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10);


//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };


//
// Payload
//
//use the max buffer size of the radio which is 32 bytes
const int max_payload_size = 32;
//buffer to send messages 
char send_msg[max_payload_size];
//buffer to receive messages and response from base station
char receive_payload[max_payload_size+1]; // +1 to allow room for a terminating NULL char

// Analog pin used to measure battery voltage
#define VccPin 0
// Digital pin connected to LED
#define ActivePin 7
//INT1 pin used to count the anemometer pulses
#define reedsw 3

//counter for anemometer
volatile int reed_sw_counter=0;
//used to toggle LED
volatile int state = 0;
//used to trigger a mainloop task
volatile int taskcntr = 8;
//used to grab the counts from the reed switch
int revolutions;

// Prototypes
void getTemperature(DeviceAddress);	// getTemperature
void getVoltage();					// getVoltage
void sendPayload();					// check if time to send payload
void getRTCTime();
void construct_msg();
float calc_windspeed();
bool TransmittMsg(char * msg);

//Setup everything
void setup(void)
{

  printf_begin();
  
#ifdef DEBUG
  Serial.begin(57600);
//  printf_begin();
  printf("\n\rRF24/examples/pingpair_dyn/\n\r");
//  printf("ROLE: %s\n\r",role_friendly_name[role]);
#endif
  //setup the voltage measurement
  analogReference(INTERNAL); 		// Set analog reference to 1.1V
  analogRead(VccPin); 			//discard first analogRead
  pinMode(ActivePin, OUTPUT);		// Set for output
  
  // Start up the library
  sensors.begin();
  // set the DS18B20 resolution 
  sensors.setResolution(Therm1, 12);

  //setup and start communications to the RTC
    Wire.begin();
    DS3231_init(DS3231_INTCN);

  //
  // Setup and configure rf radio
  //

  radio.begin();

  // enable dynamic payloads
  radio.enableDynamicPayloads();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing - this will be different for each sensor node!!!
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

#ifdef DEBUG
  radio.printDetails();
#endif  
  //attach interrupt 1 to ReedSW function to count revolutions from the anemometer to calulate wind speed
  attachInterrupt(1, ReedSW, FALLING);
  //Start a heartbeat interrupt - 500ms period - 1 second timer
  MsTimer2::set(1000, heartbeat); // 500ms period - Running at 8mhz is this becomes 1000 (500 * 2)
  MsTimer2::start();              //start the timer
 
  
}

void loop(void)
{

  if(taskcntr >= 8)
  {// every 8 seconds do this....
  
     digitalWrite(ActivePin, 1);

    //compute the reed switch - 8 seconds of average counts
     revolutions = reed_sw_counter;
     //then calculate the windspeed (kph)
     wspeedkpm = calc_windspeed();

     getVoltage();                  //get the volatge
     getTemperature(Therm1);        //get the temperature
     getRTCTime();                  //get the current time
     
     //constructs and transmitts the message     
     construct_msg();               
      //reset the tastcntr
      taskcntr = 0;
      reed_sw_counter = 0;
      
      digitalWrite(ActivePin, 0);

  }

}
//////////////////////////////////////////////////////////////////
// getTemperature
//////////////////////////////////////////////////////////////////
void getTemperature(DeviceAddress deviceAddress)
{
	sensors.requestTemperatures();
	temp1C = sensors.getTempC(deviceAddress);
//	temp1F = sensors.getTempF(deviceAddress);
#ifdef DEBUG
        Serial.println(temp1C);
#endif        
}

//////////////////////////////////////////////////////////////////////////////////
// Read analog input for VccPin averaged over NUM_SAMPLES
// Uses a running average
// Vcc is scaled with a voltage divider * 10K/(10K + 100K) so reverse
// Should be 11.0011
//////////////////////////////////////////////////////////////////////////////////
void getVoltage(){
	const byte NUM_SAMPLES = 20;
	float SumTotal=0;
	for (byte j=0;j<NUM_SAMPLES;j++){    
		SumTotal+= analogRead(VccPin);
		delay(10);
	}    		
	Vcc =  ((SumTotal/NUM_SAMPLES)*1.1/1023.0)*11.0011;
#ifdef DEBUG
        Serial.println(Vcc);
#endif        
}
//****************************************************************************************************
// INT1 function
//this is the reed switch that is attached to INT1 - used to count revololutions from the anemometer
//****************************************************************************************************
void ReedSW()
{
    reed_sw_counter++;
}

// calualte the windspeed in KPH
float calc_windspeed()
{
  
  float windspeed;
  float rpm;
  float velocity;
  float metershour;
  float c = 0.7536; // the circumfrence of the circle that makes up the reed switch
  
  rpm = revolutions * 7.5; //60/8 = 7.5 - 8 second data is accumulated 
  velocity = c * rpm;  //compute velocity - meters per min
  metershour = velocity * 60; // convert from meters per minute to meters per hour
  windspeed = metershour/1000; //convert to kilometers per hour
    
  return windspeed;
}

//*********************************************
//timer2 interrupt heartbeat - 1 second updates
//*********************************************
void heartbeat()
{
//    state = !state;  
    digitalWrite(ActivePin, state);
    taskcntr++;
}

//*******************************************
// Function to read the RTC Dtae and Time
//*******************************************
void getRTCTime()
{
  
  struct ts t;

          DS3231_get(&t);

        snprintf(tbuff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d", t.year,t.mon, t.mday, t.hour, t.min, t.sec);
#ifdef DEBUG
        Serial.println(tbuff);
#endif        
}

//****************************************************************
//Fuction to construct and send data to a base staion - uses pipes
//****************************************************************
void construct_msg()
{
 
  struct ts t;

  //get the time...
  DS3231_get(&t);
  //convert temperature to string
  dtostrf(temp1C,6,2,stemperature); 
  //convert Voltage to string
  dtostrf(Vcc,6,2,svoltage); 
  //convert wind speed to string
  dtostrf(wspeedkpm,6,2,swindspeed); 
  
 //construct and send the date, time, and bat volatge  
  snprintf(send_msg,32,"%d.%02d.%02d %02d:%02d:%02d BAT: %s", t.year,t.mon, t.mday, t.hour, t.min, t.sec, svoltage);  
  TransmittMsg(send_msg);
  
  //dealy added between transmits to allow base staion to settle
  delay(100);
  //construct and send the temperature and the windspeed
  snprintf(send_msg,32,"TMP: %s  WNDSP: %s",stemperature, swindspeed);  
  TransmittMsg(send_msg);
  
#ifdef DEBUG  
  Serial.println(send_msg);
  Serial.println(strlen(send_msg));
#endif

}

bool TransmittMsg(char * msg)
{
    bool ret = false;
    //get the length of the message  
    int msg_length = strlen(msg);
   
        // First, stop listening so we can talk.
        radio.stopListening();
#ifdef DEBUG    
        // Take the time, and send it.  This will block until complete
        printf("Now sending length %i...",msg_length);
#endif        
        radio.write( msg, msg_length );
    
        // Now, continue listening
        radio.startListening();
    
        // Wait here until we get a response, or timeout
        unsigned long started_waiting_at = millis();
        bool timeout = false;
        while ( ! radio.available() && ! timeout )
          if (millis() - started_waiting_at > 500 )
            timeout = true;
    
        // Describe the results
        if ( timeout )
        {
          ret = false;
#ifdef DEBUG          
          printf("Failed, response timed out.\n\r");
#endif          
        }
        else
        {
          // Grab the response, compare, and send to debugging spew
          uint8_t len = radio.getDynamicPayloadSize();
          radio.read( receive_payload, len );
    
          // Put a zero at the end for easy printing
          receive_payload[len] = 0;
#ifdef DEBUG    
          // Spew it
          printf("Got response size=%i value=%s\n\r",len,receive_payload);
#endif          
          ret = true;
        }

   return (ret);
 }
