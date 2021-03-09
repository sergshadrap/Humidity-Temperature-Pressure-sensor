# Humidity-Temperature-Pressure-sensor
I2c Air sensor Pressure/Humidity/Temperature based on two separate sensors HDC2080 and DPS368
Look at Images/Esensor_backside.jpg

On the back side of the sensor.
C1,C2 - capacitors for sensors stable feeding (100nf). R4,R6 - is a pull up I2C resistors (10K). R2,R3 - are not installed by default - address of HDC2080.
Both Unconnected slave address: 1000000(40H)  short R3 to GND: slave address: 1000000(40H) short R2 to VCC: slave address: 1000001(41H)

R1 - is optional select address for DPS368 (100K) empty(default) - the address 77h , 100K - 76h 
