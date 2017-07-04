# Self-balancing-robot

This is the first version of my two wheel self balancing robot. This is my first time to create a repository in GitHub. Any suggestions are welcome. 

The robot is controlled by a PID controller. MPU6050 module is a combination of accelerometer and gyroscope sensors. A complementary filter uses the information from the sensors to calculate current angle. The difference between the current angle and the desired angle serves as input for the PID controller. The output of the controller serves as input for the DC motors, which is a PWM signal.

The most important part is PID tuning. It requires some time to get a good result, i.e. the robot has relatively robust stability and small overshoot. Very large integral parameter causes instability and large overshoot; Very large derivative parameter causes very high frequency jitter.

In order to have a better control of the system, mass center should be as high as possible, and in the middle. Battery pack is normally the heaviest part, so it should be placed on the top of the robot.

In order to drive the robot, motors should have enough torque. I used the low speed, high torque dc gear motors, which has  a speed of only 200 rpm (without load) and average torque of 2kg/cm. Low speed can be compensated by large wheels.

The frame of the robot is 3mm HDF, which is cutted by a lasercutter. Two adapter parts for motors are produced by a 3D printer. M3 bolts and nuts are used to join all pieces together.

Hardware Setup:

Remote Controller Part:  
One Arduino nano microcontroller.  
One nRF24L01+ low power version module.  
One LCD screen.  
One joystick.  
Four push buttons.    
Five 220 ohm resistors.   
One potentiometer. This is used for adjusting contrast of the LCD.  
One breadboard  
Some jumper wires.  

Robot Part:  
One Arduino nano microcontroller.   
One nRF24L01+ low power version module.  
One MPU6050 module, which has an accelerometer and a gyroscope.  
One L293D motor driver IC.  
Two gear motors.   
One 12V AAA battery pack.  
One power bank.  
Some M3 bots and nuts.  
Three small breadboards.  
Some jumper wires.  




