# ESP32 flight controller
I have chosen ESP32 as a flight controller mainly because of its cost, wifi & BLE capabilities, and 80MHz clock speed which is 5 times faster than Arduino. 

## Code files
1. Anglemode_flightcontroller_ver3 -  This is the complete code for ESP32 flight controller. Make sure to give an initial 10s calibration time for the quadcopter to calibrate Gyro on a spirit level.
2. receiver_pwm_esp32 - This code is independently to test the receiver & transmitter signals on serial monitor
3. measure_angles_mpu - IMU angle checker
4. Motors calibration - The motor's ESCs have to be calibrated before flight, such that they start simultaneously.

## Youtube Video link
https://youtu.be/BcPTe-hTJGc?si=5ig9wEDCqvCX-sgO
