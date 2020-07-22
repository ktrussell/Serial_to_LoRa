// LoRa Receive Program Version 3.1
// Serial_to_LoRa_RX_V3-1.ino
// Copyright Kenny Trussell
// http://kenny.trussell.biz
//
// Shared under the MIT License
//
// Version 3.2.1 Changes:
//  Bug fix: Fixed the activity indicator function
//
// Version 3.2 Changes:
//  Created an Activity indicator on pin 12: set low if no packets are being
//  received, set high if packets are being received
//
// Version 3.1 Changes:
//  Added output of RSSI to serial monitor for antenna testing
//  Changed LoRa Bandwidth to 62.5kbps
//
// This version collects incoming data until no LoRa packets have been
//  received for RFWAITTIME milliseconds and then sends all received data to
//  the serial port. This is a companion to the transmit program:
//  Serial_to_Lora_TX

// Target Hardware: Adafruit Feather M0 LoRa module:
//  https://www.adafruit.com/product/3178
//
// Hardware Notes:
//  I powered the feather via the Vbat pin. This feeds through an MBR120 diode
//  (Vf is 340mV to 530mV depending on which variant of MBR120 Adafruit used) to
//  an AP2112K-3.3 voltage regulator. The max dropout voltage for the AP2112K-3.3
//  is 400mV. So minimum at Vbat to account for MBR120 Vf and AP2112K-3.3 droput
//  is 3.3V+0.53+0.4 = 4.23V. The maximum recommended input voltage for the
//  regulator is 6V. So desired Vbat range is 4.23-6V. In addition, there is
//  an MCP73881/2 LiPo charge controller connected to Vbat. It is designed to
//  charge a battery connected to Vbat when power is present on the USB port.
//  In my case, I do not want current ever to flow out of the Vbat terminal.
//  To disable the charge control circuit, we need to remove the programming
//  resistor R8 from the board. This is a tiny surface mount resistor. The silk
//  screen label in the Eagle PCB file for R8 is actually offset to the left of
//  the resistor. It is easy to be misled and remove the wrong resistor.
//  (experience talking!) R8 is the component just above the GND pin on the
//  board. It is just to the left of the larger 32.768 Crystal.
//
//  After the above modification, the board should be powered through Vbat
//   with a voltage between 4.23 and 6.0.

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

#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <RH_RF95.h>

#define ACTIVITY_PIN 12 //held HIGH if data being received, otherwise LOW

// Max size of data burst we can handle (5 full RF buffers)-arbitrarily large
#define BUFLEN (5*RH_RF95_MAX_MESSAGE_LEN)

// Maximum milliseconds to wait for next LoRa packet
//  Up to 300, program sends each packet to GPS. 350 causes it to get multiple
//  packets. At 350, sometimes it is getting two bursts together, i.e. about
//  1200 bytes, when a burst is 600 bytes.
#define RFWAITTIME 1 //1 will cause packet to be sent to GPS immediately

// For feather m0
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

// LED is turned on at 1st LoRa reception and off when nothing else received.
//  It gives an indication of how long the incoming data stream is.
#define LED 13

unsigned long lastActCheckTime; //holds last millis() value when activity was checked

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(ACTIVITY_PIN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(ACTIVITY_PIN, LOW); //Initialize to no activity

  Serial.begin(115200);
//  while (!Serial) { //Waits for the Serial Monitor
//    delay(1);
//  }

  Serial2.begin(115200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  delay(100);

  Serial.println("Feather LoRa RX");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  for(int ii=0; ii<100; ii++) { Serial.println("TEST"); }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  //  Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  //  If you are using RFM95/96/97/98 modules which uses the PA_BOOST
  //  transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  rf95.setSignalBandwidth(62500L);

  lastActCheckTime = millis(); //initialize time for activity check
}

void loop()
{
  uint8_t buf[BUFLEN];
  unsigned buflen;

  uint8_t rfbuflen;
  uint8_t *bufptr;
  unsigned long lastTime, curTime;
  static bool RX_Activity;  //Set to true if we are receiving RF packets

  bufptr = buf;
  buflen = 0;

  if (rf95.available())
  {
    RX_Activity = true; //Indicate that we have activity
    digitalWrite(LED, HIGH);
    rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
    if(rf95.recv(bufptr, &rfbuflen))
    {
      bufptr += rfbuflen;
      lastTime = millis();

      while(((millis()-lastTime) < RFWAITTIME) && ((bufptr - buf) < (BUFLEN - RH_RF95_MAX_MESSAGE_LEN))) //Time out or buffer can't hold anymore
      {
        if (rf95.available())
        {
          rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
          if(rf95.recv(bufptr, &rfbuflen))
          {
            bufptr += rfbuflen;
            lastTime = millis();
          }
          else
          {
            Serial.println("Receive failed");
          }
        }
      }
    }
    else
    {
      Serial.println("Receive failed");
    }

    buflen = (bufptr - buf);  // Total bytes received in all packets
    
    Serial.print(rf95.lastRssi(), DEC);
    Serial.print(" ");
    Serial.println(buflen);   // For debugging
    Serial2.write(buf, buflen); //Send data to the GPS
    digitalWrite(LED, LOW);
  }

  //Activity indicator output
  if(millis()-lastActCheckTime > 1500)
  {
    if(RX_Activity)
    {
      digitalWrite(ACTIVITY_PIN, HIGH);  //Indicate that we received data
    }
    else
    {
      digitalWrite(ACTIVITY_PIN, LOW); //Indicate no data received
    }
    RX_Activity = false;  //set flag to false - receive activity (if any) will set it to true
    lastActCheckTime = millis(); //reset time
  }
}
