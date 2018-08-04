# Serial_to_LoRa
Transmit and receive programs for Adafruit Feather M0 LoRa module to transmit one way data over LoRa. I use these routines on the boards with a U-blox C94-M8P evaluation kit to send RTK correction data from base GPS to rover GPS for an autonomous lawn mower.

Using Feather M0 Lora with a Ublox C94-M8P evaluation kit:

Rover:
I am powering the board with its Vbat input with 5V from the Servo power rail on the Pixhawk.

Base:
The C94-M8P can be powered with a 3.7V – 20V supply. Since the Feather Vbat can use a voltage around 3.7-4.2, I am powering them from the same supply. The power supply can be anything from 4.1V – 6V. 

Both boards:
I powered the feather via the Vbat pin. This feeds through an MBR120 diode (pretty low Vf) to an AP2112K-3.3 voltage regulator. The maximum recommended input voltage for the regulator is 6V. In addition, there is an MCP73881/2 LiPo charge controller connected to Vbat. Normally a battery is connected to Vbat and when power is presented at the USB port, the charge controller will charge the battery. In my case, I do not want current flow to ever occur back out the Vbat terminals. To disable the charge control circuit, we need to remove the programming resistor R8 from the board. This is a tiny surface mount resistor. The silk screen label in the Eagle PCB file for R8 is actually offset to the right of the resistor. It is easy to be misled and remove the wrong resistor! R8 is the component just above the GND pin on the board. It is just to the left of the larger 32.768 Crystal. (I removed the wrong component my first time!) See the included jpg image of the board showing the correct resistor to remove.

After the above modification, the board should be powered through Vbat with a voltage between 4.1 and 6.0. The 4.1V lower limit comes from [3.3V + 0.4V (max Dropout of MCP73881/2) + 0.4V (max Vf across MBR120)]. The upper limit is the max recommended input of the MCP73881/2.
 
You must disable the onboard radio modem on the C94-M8P boards as well. Refer to the documentation for the board on how to do that.
