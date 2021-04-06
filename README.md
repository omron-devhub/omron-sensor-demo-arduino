# omron-sensor-demo-arduino
It is a sample projects for OMRON Sensor demo kit with
evaluation board **2JCIE-EV01-AR1** and some Arduino boards.

OMRON has the various sensors.
This firmware can work on Arduino and OMRON PC soft can display data and store the data by CSV file.
Contact to OMRON Sales member about PC soft.

Sensor
- OMRON D6T series (Thermal sensor) 
- OMRON D6F series (Flow sensor) 
- OMRON D6F-PH series (Differential Pressure sensor) 
- OMRON B5W-LB series (Light Conversion sensor) 
- OMRON 2SMPP series (Gauge Pressure sensor)
- OMRON 2SMPB series (Absolute Pressure sensor)  

## Description
this Arduino sample projects for acquiring data from sensors on 2JCIE-EV01.
sample projects output the sensor data to USB-Serial ports.

| example | sensor type                | board |
|:-------:|:---------------------------|:-----------------------|
| all-sensor  | D6T / D6F / D6F-PH / B5W-LB / 2SMPP / 2SMPB | Arduino MKR-WiFi1010|

## Installation
see `https://www.arduino.cc/en/guide/libraries`

### Install from Arduino IDE
1. download .zip from this repo [releases](releases)
    or [master](archive/master.zip) .

2. Import the zip from Arduino IDE

3. Then, you can see the samples in `File >> Examples` menu.

4. Select examples for your favorite sensors, build and program to boards.

### Manual install
1. download this repo

    ```shell
    $ git clone https://github.com/omron-devhub/omron-sensor-demo-arduino
    ```

2. launch Arduino-IDE and select our sketch to load.
3. build and program to boards.


## Dependencies
None

## Links
- [Arduino sample for D6T on 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/d6t-2jcieev01-arduino)
- [Arduino sample for D6F on 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/d6f-2jcieev01-arduino)
- [Arduino sample for B5W on 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/b5w-2jcieev01-arduino)


## Licence
Copyright (c) OMRON Corporation. All rights reserved.

Licensed under the MIT License.

