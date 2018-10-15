# nova-satellite-code
Nova 1 is a low cost picosatellite developed by students at PrepaTec Colima. 
This is the control code that manages the flight operations. It runs on an Arduino Pro Mini, and its tasks are to read the variety of sensors onboard, form the binary telemetry packets and manage the radio downlink to ground control.

## Telemetry packet format
The main objectives of this packet format are efficiency and ease of packing and unpacking. We chose to develop a custom binary format where we allocate our precious bandwidth to the data we care about. 

For simplicity, we only have one type of packet. It might be more efficient to have different types of packets that contain sensors with similar update rates, but in practice most of our sensors refresh at rates higher than our downlink rate. 

Our only packet type has a size of 296 bits (37 bytes), and we send it every ≈363ms, which makes the bitrate 815.3 bps. That's approximately a 2.75Hz update rate, which is more than enough for our needs, since most of the telemetry data is informative and intended for post-mission analysis.

Here's the breakdown:

|Field|Size (bits)|Data type|Unit|
|:---:|:---------:|:-------:|:--:|
|Packet number|16|Unsigned int|Packets|
|Time since boot|24|Unsigned int|Milliseconds|
|BMP temperature|16|Signed int|1/10 °C|
|BMP atmospheric pressure|19|Unsigned int|Pascals|
|Number of GPS satellites in use|4|Unsigned int|Amount of GPS satellites|
|GPS fix status|1|Bool|---|
|Acelerometer X axis|16|See datasheet|m/s²|
|Acelerometer Y axis|16|See datasheet|m/s²|
|Acelerometer Z axis|16|See datasheet|m/s²|
|Magnetic declination GPS|16|Signed integer|1/100 °|
|Latitude GPS|32|Signed integer|1/10⁵ °|
|Longitude GPS|32|Signed integer|1/10⁵ °|
|Altitude GPS|24|Unsigned integer|Centimeters|
|Speed GPS|24|Signed integer|cm/s|
|Magnetometer X axis|12|See datasheet|Gauss|
|Magnetometer Y axis|12|See datasheet|Gauss|
|Magnetometer Z axis|12|See datasheet|Gauss|
|Padding|4|
|**TOTAL**|296|

### Byte-by-byte breakdown
|Addresses|Content|
|:-------:|:-----:|
| 0x00 - 0x01  | Packet number|

This one's easy: we need a unique, monotonically increasing ID for each packet. This way we can identify any gap in the packets and it also allows us to merge the data from two receivers at different locations. Like most of the data described here, it's sent LSB-first, since AVR is a little-endian architecture, to facilitate packing and unpacking.
With 2^16 available packet numbers and assuming a packet update rate of 3Hz, the counter would overflow 6 hours 4 minutes into the mission. Overflow doesn't matter since we can disambiguate packets using the next field - time since boot.

|Addresses|Content|
|:-------:|:-----:|
| 0x02 - 0x04  | Time since boot|

We also need a high resolution time reference to be able to locate important mission events in context. We allocated 24 bits to this function. Since we chose milliseconds as our unit, the counter would overflow 4 hours 39 minutes into the mission. 

|Addresses|Content|
|:-------:|:-----:|
| 0x05 - 0x06  | BMP temperature |
| 0x07 - 0x09[0:2]  | BMP atmospheric pressure |

Why measure atmospheric pressure? Because it allows us to determine our altitude in a precise way. To get altitude from atmospheric pressure, we need to determine temperature first, since the pressure at a given altitude depends on a couple of factors, including temperature. We'll use Bosch's BMP180 sensor for both tasks: for temperature, we pull the data according to our update rate, apply the calibration coefficients stored in the chip's EEPROM to obtain true temperature and send it, LSB-first, as a 16-bit signed integer with a resolution of 1/10 of a degree C. 

For atmospheric pressure, we read the 19 bits of resolution from our BMP180, and then use the calibration data to obtain true pressure. Our units are Pascals, and we don't need any multipliers because the sensor has a typical accuracy of ± 12 Pascals. With regards to the size, 19 unsigned bits are enough for pressures from 0 to 524,288 Pa, which means we can obtain altitude coming all the way from the edge of the atmosphere, passing through mean sea level at 101,325 Pa and overflowing around 16 km inside the Earth. More than enough, since the world's deepest borehole reaches 12.34 km.


With regard to temperature, the variable would overflow at ±3276.8 °C, but if we reach any of those temperatures, the overflow here would be the minor of our concerns. Especially since temperatures below -273.15 °C are physically impossible.


|Addresses|Content|
|:-------:|:-----:|
| 0x09[3:6] | Number of GPS satellites in use |

Four bits, 16 different values. Since our chip supports the usage of up to 12 satellites at once, we have enough space. 

|Addresses|Content|
|:-------:|:-----:|
| 0x09[7] | GPS fix status |

Self-explanatory. 


|Addresses|Content|
|:-------:|:-----:|
| 0x0A - 0x0B | Accelerometer X axis|
| 0x0C - 0x0D | Accelerometer Y axis|
| 0x0E - 0x0F | Accelerometer Z axis|

TODO

|Addresses|Content|
|:-------:|:-----:|
| 0x10 - 0x11 | Magnetic declination GPS|

TODO

|Addresses|Content|
|:-------:|:-----:|
| 0x12 - 0x15 | Latitude GPS|
| 0x16 - 0x19 | Longitude GPS|

TODO

|Addresses|Content|
|:-------:|:-----:|
| 0x1A - 0x1C | Altitude GPS|

TODO

|Addresses|Content|
|:-------:|:-----:|
| 0x1D - 0x1F | Speed GPS|

TODO

|Addresses|Content|
|:-------:|:-----:|
| 0x20 - 0x21[0:3] | Magnetometer X axis|
| 0x21[4:7] - 0x22 | Magnetometer Y axis|
| 0x23 - 0x24[0:3] | Magnetometer Z axis|

TODO