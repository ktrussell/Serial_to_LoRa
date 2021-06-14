# Serial_to_LoRa
Transmit and receive programs for Adafruit Feather M0 LoRa module to transmit one way data over LoRa. I have used these routines previously with a U-blox C94-M8P evaluation kit to send RTK correction data (RTCM3) from base GPS to rover GPS for an autonomous lawn mower based on Ardupilot running on a Pixhawk. I have upgraded to U-blox C099-F9P eval boards and Ardusimple SimpleRTK2B boards (Either will work.)

I discovered on July 1, 2020, that I could run the LoRa modules at a bandwidth of 62.5kbps instead of 125kbps. I made that change which theoretically will give more range. In testing, I did not notice a difference.

**Modifications to both Rover and Base Feather M0 (for my situation, at least):**
I powered the feather via the Vbat pin from another board (host) in my system (the GPS for instance). The board has an MCP73881/2 LiPo charge controller connected to Vbat. It is designed to charge a battery connected to Vbat when power is present on the USB port. I have had a couple of Feather failures which I believe were caused by the Feather back-powering the host board when I had a USB cable connected to the Feather (for programming) but I did NOT power up the host board. I am not completely sure, but to keep this from happening, I inserted a diode, such as a 1N4007 in the +5V signal from the host.

**Using Feather M0 Lora with Ublox C099-F9P evaluation boards:**

Rover:
I am powering the Feather board with its Vbat input from the 5V Servo power rail on the Pixhawk through a diode.

Base:
I am powering the Feather Vbat from the 5Vout pin of the C099-F9P board through a diode.

**Using Feather M0 Lora with a Ublox C94-M8P evaluation kit:**

Rover:
I am powering the Feather board with its Vbat input from the 5V Servo power rail on the Pixhawk through a diode.

Base:
The C94-M8P can be powered with a 3.7V – 20V supply. I am powering the C94-M8P and the Feather from a 5V wall wart power supply.

You must disable the onboard radio modem on the C94-M8P boards as well. Refer to the documentation for the board on how to do that.


Kenny Trussell
http://kenny.trussell.biz
