// LoRa Transmit Program Version 3
// Serial_to_LoRa_TX_V3.ino
// Copyright Kenny Trussell
// Shared under the MIT License
//
// This version sends 200 byte packets, ignoring any RTCM protocol boundaries
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

#define RTCM_START 0xd3
#define BUFLEN 1000 //max size of data burst we can handle
#define SER_TIMEOUT 200 //Timeout in millisecs for reads into buffer from serial - needs to be longer than bit time at our baud
                        //rate and any other delay between packets from GPS

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

// The LED is turned on when 1st byte is received from the serial port. It is turned off after the last byte is transmitted over LoRa.
#define LED 13

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

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  Serial2.setTimeout(SER_TIMEOUT);

}

void loop()
{
  uint8_t buf[BUFLEN];
  uint8_t* bufptr;
  int bytesRead;
  int bytesLeft;

  if (Serial2.available())
  {
    digitalWrite(LED, HIGH);
    bytesRead = Serial2.readBytes((char *) buf, BUFLEN); // If timeout is set properly, this should read an entire burst from
	                                                     // the M8P.
    Serial.println(bytesRead);  //This is useful for debugging if the serial monitor is open. It doesn't add much overhead to the
    							//program.

    // Process the entire received buffer (buf) in individual 200 byte packets.
    //  bufptr points to the start of the bytes to be transmitted. It is moved through buf until all bytes are transmitted.
    bufptr = buf;
    bytesLeft = bytesRead;
    while(bytesLeft > 0)
    {
      if( bytesLeft < 200)
      {
        rf95.send(bufptr, bytesLeft);
        bytesLeft = 0;
      }
      else
      {
        rf95.send(bufptr, 200);
        bufptr += 200;
        bytesLeft -= 200;
      }
    }
    digitalWrite(LED, LOW);
  }
}
