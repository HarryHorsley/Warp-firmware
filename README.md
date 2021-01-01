Name: Harry Horsley

College: Jesus 

CRSid: hslh2

The projects focuses on optimizing the light incident upon the surface of a solar panel by rotating the panel through 1-axis of freedom. This is achieved with two light sensors and a motor.

The files descriptions are as follows:

warp-kl03-ksdk1.1-boot.c     -->         Contains a stripped down version of "warp-kl03-ksdk1.1-boot.c" (found on GitHub) with various new functions and my implementation of the light tracking system. It also contains attempts to get the PWM working.

devINA219.*                           -->         Device driver for the INA219 current monitor, based off of the devMMA8451Q.* files.

devVEML7700.*                     -->         Device driver for the VEML7700 lux sensor, based off of the devMMA8451Q.* files.

devMultiplex.*                        -->         Device driver for the TCA9548A 1 to 8 Multiplexer, based off of the devMMA8451Q.* file.
