// LoRa Transmit Program Version 4.2
// Serial_to_LoRa_TX_V4-2.ino
// Copyright Kenny Trussell
// http://kenny.trussell.biz
//
// Shared under the MIT License
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

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
//  while (!Serial) { //Waits for the Serial Monitor
//    delay(1);
//  }

  Serial2.begin(57600);

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
  Serial2.setTimeout(SER_TIMEOUT);

  sendSize = rf95.maxMessageLength();

  int countdownMS = Watchdog.enable(3000); //Set watchdog timer to 3.0 seconds.
}

void loop()
{
  uint8_t buf[BUFLEN];
  uint8_t* bufptr;
  int bytesRead;
  int bytesLeft;

  Watchdog.reset();

  if (Serial2.available())
  {
    digitalWrite(LED, HIGH);

    // If timeout is set properly, this should read an entire burst from the M8P.
    bytesRead = Serial2.readBytes((char *) buf, BUFLEN);

    Serial.println(bytesRead);  //For debugging purposes

    // Process the entire received buffer (buf) in individual sendSize byte
    //  packets. bufptr points to the start of the bytes to be transmitted.
    //  It is moved through buf until all bytes are transmitted.
    bufptr = buf;
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
    digitalWrite(LED, LOW);
  }
}
