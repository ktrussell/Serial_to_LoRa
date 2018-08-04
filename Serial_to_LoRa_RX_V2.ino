// LoRa Receive Program Version 2
// Serial_to_LoRa_RX_V2.ino
// Copyright Kenny Trussell
// http://kenny.trussell.biz
//
// Shared under the MIT License
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
//  (pretty low Vf) to an AP2112K-3.3 voltage regulator. The maximum
//  recommended input voltage for the regulator is 6V. In addition, there is
//  an MCP73881/2 LiPo charge controller connected to Vbat. Normally a battery
//  is connected to Vbat and when power is presented at the USB port, the
//  charge controller will charge the battery. In my case, I do not want
//  current flow to ever occur back out the Vbat terminals. To disable the
//  charge control circuit, we need to remove the programming resistor R8
//  from the board. This is a tiny surface mount resistor. The silk screen
//  label in the Eagle PCB file for R8 is actually offset to the left of the
//  resistor. It is easy to be misled and remove the wrong resistor! R8 is the
//  component just above the GND pin on the board. It is just to the left of
//  the larger 32.768 Crystal. (I removed the wrong component my first time!)
//
//  After the above modification, the board should be powered through Vbat
//   with a voltage between 4.1 and 6.0.

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

#define BUFLEN (5*RH_RF95_MAX_MESSAGE_LEN) //max size of data burst we can handle - (5 full RF buffers) - just arbitrarily large
#define RFWAITTIME 350 //maximum milliseconds to wait for next LoRa packet - used to be 600 - may have been too long

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

// LED is turned on at 1st LoRa reception and off when nothing else received. It gives an indication of how long the incoming data stream is.
#define LED 13

void setup() {
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

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

}

void loop() {
uint8_t buf[BUFLEN];
unsigned buflen;

uint8_t rfbuflen;
uint8_t *bufptr;
unsigned long lastTime, curTime;

  bufptr = buf;
//  Serial.println("----"); Serial.print((long) buf); Serial.print(" "); Serial.println((long) bufptr); //Temp for debug

  if (rf95.available())
  {
    digitalWrite(LED, HIGH);
    rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
    if(rf95.recv(bufptr, &rfbuflen))
    {
//        Serial.println(rfbuflen);
//      Serial.print("A: "); //temp for debugging
//      Serial.println(rfbuflen); //temp for debugging
//      Serial.println((unsigned char) *bufptr, HEX);
      bufptr += rfbuflen;
//      Serial.print((long) buf); Serial.print(" "); Serial.println((long) bufptr); //Temp for debug
      lastTime = millis();

      while(((millis()-lastTime) < RFWAITTIME) && ((bufptr - buf) < (BUFLEN - RH_RF95_MAX_MESSAGE_LEN))) //Time out or buffer can't hold anymore
      {
        if (rf95.available())
        {
          rfbuflen = RH_RF95_MAX_MESSAGE_LEN;
          if(rf95.recv(bufptr, &rfbuflen))
          {
//          Serial.println(rfbuflen);
//          Serial.println(rfbuflen); //temp for debugging
//            Serial.println((unsigned char) *bufptr, HEX);
            bufptr += rfbuflen;
//            Serial.print((long) buf); Serial.print(" "); Serial.println((long) bufptr); //Temp for debug
            lastTime = millis();
          }
          else
          {
            Serial.println("Receive failed");
          }
        }
      }
    }
    else {
      Serial.println("Receive failed");
    }

      buflen = (bufptr - buf); 		//Total bytes received in all packets
      Serial.println(buflen);		//This is useful for debugging if the serial monitor is open. It doesn't add much overhead to
      								//the program.
      Serial2.write(buf, buflen);	//Send data to the GPS
//      Serial.println("C");
//      Serial.println(buflen);
//      Serial.println("----");
//      Serial.write(buf, buflen); //temp for debugging
      digitalWrite(LED, LOW);
  }
}
