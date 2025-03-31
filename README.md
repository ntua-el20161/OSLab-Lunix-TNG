# Lunix TNG Device Driver
In this project we implemented a character device driver for linux. The driver receives raw data from a censor network and exports them to different devices in userspace based on the type of the measurement and the censor from which the measurement originates. So, instead of one and only /dev/ttyUSB1 existing, there are 3 devices for each censor. For example, for the first censor there are /dev/lunix0-temp, /dev/lunix0-light, /dev/lunix0-batt.



