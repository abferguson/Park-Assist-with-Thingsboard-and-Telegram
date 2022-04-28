# Park-Assist-with-Thingsboard-and-Telegram
An ESP8266-based device using ultrasonic sensor to help driver park car at the correct position

I ran across other maker's park assist devices and decided it might be useful for my sister. She has a tendency to park too close to the back wall of her garage resulting in damage to the wall. It is useful and it does work. I made two of these for myself.

My goal was to use the HC-SR04 ultrasonic sensor to make the measurements coupled with a 5mm LED and an LED strip to provide visual feedback of distance:
  1) Turn the LED on when the car was within range of the sensor
  2) Blink the LED white at a faster rate the closer the car was to the sensor
  3) Change the LED and WS2812B LED strip color to red, with no blinking, when the car reaches the park position.
  4) When approaching or departing, display LED strip LEDs as a function of distance: the closer the vehicle the fewer LEDs lit relative to center LED.
  5) Provide a momentary contact button on the unit to provide distance setting capability.
  6) Store the park position distance onboard the unit, using LittleFS library, in case of reboot or power failure.
  
The park assist unit is also integrated with ThingsBoard and Telegram as options.  Via ThingsBoard, the unit can be monitored for the latest network parameters, sensor readings, reboots and for errors.  Telegram notifications can be enable through Thingsboard.  The park stop distance used by the unit can also be changed through ThingsBoard.

Telegram is used to send a message to the user when the vehicle has been out of range for more than a minute. This is a potential security feature.

Additional LED effects:
After five seconds of being in the 'park' position, the 5mm LED brighness is reduced.  Additionally, a random Neopixel animation is executed for amusement for a few seconds.  the Neopixels are then turned off.

Firmware environment: Arduino IDE

Hardware: (see Fritzing file for circuit)
 1 x NodeMCU v1.0 (ESP-12E) microcontroller
 
 1 x 5mm RGB LED
 
 3 x 470 Ohm resistor (for RGB LED)
 
 1 x 10k Ohm resistor (momentary switch)
 
 1 x momentary contact tactile button
 
 1 x HC-SR04 ultrasonic sensor
 
 1 x WS2912b LED strip (I use 21 LEDs. ESP8266 can power these).
 
 1 x 470 Ohm resistor (WS2912b data line)
 
 1 x 4 channel level converter (HC-SR04 trigger and echo lines, WS2812b data line)
 
 1 x 1000 uF capacitor (for WS2812B LED strip)
 
