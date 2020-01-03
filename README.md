# Serial_to_LoRa
Transmit and receive programs for Adafruit Feather M0 LoRa module to transmit one way data over LoRa. I have used these routines previously with a U-blox C94-M8P evaluation kit to send RTK correction data from base GPS to rover GPS for an autonomous lawn mower based on Ardupilot running on a Pixhawk. I have upgraded to U-blox C099-F9P eval boards and Ardusimple SimpleRTK2B boards (Either will work.)

**Modifications to both Rover and Base Feather M0 (for my situation, at least):**
I powered the feather via the Vbat pin. This feeds through an MBR120 diode (Vf is 340mV to 530mV depending on which variant of MBR120 Adafruit used) to an AP2112K-3.3 voltage regulator. The max dropout voltage for the AP2112K-3.3 is 400mV. So minimum at Vbat to account for MBR120 Vf and AP2112K-3.3 droput is 3.3V+0.53+0.4 = 4.23V. The maximum recommended input voltage for the regulator is 6V. So desired Vbat range is 4.23-6V. In addition, there is an MCP73881/2 LiPo charge controller connected to Vbat. It is designed to charge a battery connected to Vbat when power is present on the USB port. In my case, I do not want current ever to flow out of the Vbat terminal. To disable the charge control circuit, we need to remove the programming resistor R8 from the board. This is a tiny surface mount resistor. The silk screen label in the Eagle PCB file for R8 is actually offset to the left of the resistor. It is easy to be misled and remove the wrong resistor. (experience talking!) R8 is the component just above the GND pin on the board. It is just to the left of the larger 32.768 Crystal.

After the above modification, the board should be powered through Vbat with a voltage between 4.23 and 6.0.
 
**Using Feather M0 Lora with Ublox C099-F9P evaluation boards:**

Rover:
I am powering the Feather board with its Vbat input from the 5V Servo power rail on the Pixhawk.

Base:
I am powering the Feather Vbat from the 5Vout pin of the C099-F9P board.

**Using Feather M0 Lora with a Ublox C94-M8P evaluation kit:**

Rover:
I am powering the Feather board with its Vbat input from the 5V Servo power rail on the Pixhawk.

Base:
The C94-M8P can be powered with a 3.7V – 20V supply. The Feather Vbat (after modifications described below) should have Vbat connected to 4.23 to 6.0V. I am powering them from a 5V wall wart power supply.

You must disable the onboard radio modem on the C94-M8P boards as well. Refer to the documentation for the board on how to do that.


Kenny Trussell
http://kenny.trussell.biz
