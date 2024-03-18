// LoRa Transmit Program Version 4.4
// Serial_to_LoRa_TX_V4-4.ino
// Copyright Kenny Trussell
// http://kenny.trussell.biz
//
// Shared under the MIT License
//
// Version 4.4 Changes:
//  -Added ability to enable a small heartbeat packet once a second if no other
//   packets are to be sent. This allows the receiver to communicate to the
//   host processor that the transmitter is in range even if no RTCM3 packets
//   are coming in. To enable the heartbeat feature, remove the "//" in the
//   statement below which reads: "// #define USE_HEARTBEAT".
//  -Updated Hardware section to discuss diode
//
// Version 4.3 Change:
// Changed LoRa Bandwidth to 62.5kbps
//
// Rev 4.2 Changes
// 1. This version sends the maximum message length supported by the RF95 driver as
//    determined by a call to RF95.maxMessageLength().
// 2. Lengthened watchdog timeout.
//
// Rev 4.1
// Added watchdog timeout to fix issue when unit hangs and updated
// hardware notes in the comments below.
//
// Target Hardware: Adafruit Feather M0 LoRa module:
//  https://www.adafruit.com/product/3178
//
// Hardware Notes:
//  I found that if the Feather is powered from 5V from another board, that if
//  the other board is not powered and you insert a USB cable from a PC into
//  the Feather (for programming for instance), that the Feather backfeed 5V
//  to the other board from the LiPo charge circuit. To
//  prevent this, add a diode in the 5V line from the other board. Any diode with a Vf of 1V or
//  less and a current rating on the order of 200mA should work fine. A
//  1N4001 throught 1N4007 device is a good choice.

/* MIT License
 * Copyright (c) 2018 Kenny Trussell
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// #define USE_HEARTBEAT

#include <Arduino.h>        // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_SleepyDog.h>

#define RTCM_START 0xd3
#define BUFLEN 2000    //max size of data burst from GPS we can handle
#define SER_TIMEOUT 50 //Timeout in millisecs for reads into buffer from serial
                       // - needs to be longer than byte time at our baud rate
                       // and any other delay between packets from GPS

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// We will use Serial2 - Rx on pin 11, Tx on pin 10
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// The LED is turned on when 1st byte is received from the serial port. It is
//  turned off after the last byte is transmitted over LoRa.
#define LED 13

int sendSize; //Will hold result of a call to RF95.maxMessageLength(). This
              // value will be the length of packet we send.

static bool SendHeartBeat;  //Set to true if it is time to send a heartbeat packet
unsigned long lastHeartBeatTime; //holds last millis() value when heartbeat was sent

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
//  while (!Serial) { //Waits for the Serial Monitor
//    delay(1);
//  }

  Serial2.begin(115200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  delay(100);

  Serial.println("Feather LoRa TX");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  //  Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  //  transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  rf95.setSignalBandwidth(62500L);
//  rf95.setCodingRate4(6);

  Serial2.setTimeout(SER_TIMEOUT);

  sendSize = rf95.maxMessageLength();

  lastHeartBeatTime = millis(); //initialize time for activity check

  int countdownMS = Watchdog.enable(3000); //Set watchdog timer to 3.0 seconds.
}

void loop()
{
  uint8_t buf[BUFLEN];
  uint8_t* bufptr;
  int bytesRead;
  int bytesLeft;

  Watchdog.reset();


#ifdef USE_HEARTBEAT
  //Timer to determine if time to send a heartbeat
  if(millis()-lastHeartBeatTime > 1500)
  {
    SendHeartBeat = true;
    lastHeartBeatTime = millis(); //reset time
  }
#endif

#ifndef USE_HEARTBEAT
  SendHeartBeat = false;
#endif

  if (Serial2.available() || (SendHeartBeat==true))
  {
    digitalWrite(LED, HIGH);

    // If timeout is set properly, this should read an entire burst from the GPS.
    bytesRead = Serial2.readBytes((char *) buf, BUFLEN);

    Serial.println(bytesRead);  //For debugging purposes

    // Process the entire receive buffer (buf) in individual sendSize byte
    //  packets. bufptr points to the start of the bytes to be transmitted.
    //  It is moved through buf until all bytes are transmitted.
    bufptr = buf;

#ifdef USE_HEARTBEAT
    // If nothing received, we here only to send a heartbeat
    if(bytesRead == 0)
    {
      buf[0] = 'H'; //Put our heartbeat character in the buffer
      bytesRead = 1; //Indicate there is a character in the buffer
    }
#endif

    bytesLeft = bytesRead;
    while(bytesLeft > 0)
    {
      if( bytesLeft < sendSize)
      {
        rf95.waitPacketSent();
        rf95.send(bufptr, bytesLeft);
        bytesLeft = 0;
      }
      else
      {
        rf95.waitPacketSent();
        rf95.send(bufptr, sendSize);
        bufptr += sendSize;
        bytesLeft -= sendSize;
      }
    }

#ifdef USE_HEARTBEAT
    //Reset the heartbeat flag and timer
    SendHeartBeat = false;
    lastHeartBeatTime = millis(); //reset time
#endif

    digitalWrite(LED, LOW);
  }
}
