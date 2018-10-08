# nova-satellite-code
Nova 1 is a low cost picosatellite developed by students at PrepaTec Colima. 
This is the control code that manages the flight operations. It runs on an Arduino Pro Mini, and its tasks are to read the variety of sensors onboard, form the binary telemetry packets and manage the downlink to ground control.

## Telemetry packet format
The main objectives of this packet format are efficiency and ease of packing and unpacking. We chose to develop a custom binary format where we allocate our precious bandwidth to the data we care about. 

For simplicity, we only have one type of packet. It might be more efficient to have different types of packets that contain sensors with similar update rates, but in practice most of our sensors refresh at rates higher than our downlink rate. 

Our only packet type has a size of 400 bits (50 bytes), and we send it every 200ms. That's a 5Hz update rate, which is more than enough for our needs, since most of the telemetry data is informative and intended for post-mission analysis.

Here's the breakdown:

|Field|Size (bits)|Data type|Unit|
|-----|-----------|---------|----|
|Packet number|24|Unsigned int|Packets|
|Time since boot|32|Unsigned int|Milliseconds|
|BMP temperature|16|Signed int|1/10 °C|
|BMP atmospheric pressure|19|See datasheet|hPa|
|Number of satellites in use|4|Unsigned int|Amount of GPS satellites|
|GPS fix status|1|Bool|---|
|Magnetometer X axis|16|See datasheet|Gauss|
|Magnetometer Y axis|16|See datasheet|Gauss|
|Magnetometer Z axis|16|See datasheet|Gauss|
|Acelerometer X axis|16|See datasheet|m/s²|
|Acelerometer Y axis|16|See datasheet|m/s²|
|Acelerometer Z axis|16|See datasheet|m/s²|
|Gyroscope X axis|16|Signed integer|1/10 °|
|Gyroscope Y axis|16|Signed integer|1/10 °|
|Gyroscope Z axis|24|Signed integer|1/1000 °|
|Latitude GPS|32|Signed integer|1/10⁵ °|
|Longitude GPS|32|Signed integer|1/10⁵ °|
|Altitude GPS|24|Unsigned integer|Centimeters|
|Speed GPS|24|Signed integer|cm/s|
|Time GPS|24|Unsigned integer| 1/100 second|
|Magnetic declination GPS|16|Signed integer|1/100 °|
|Error correction|16|Reed-Solomon data|---|

### Byte-by-byte breakdown
|  Addresses | Content  |
|----------|---|
| 0x00 - 0x02  | Packet number - LSB to MSB |

This one's easy: we need a unique, monotonically increasing ID for each packet. This way we can identify any gap in the packets and it also allows us to merge the data from two receivers at different locations. Like most of the data described here, it's sent LSB-first, since AVR is a little-endian architecture, to facilitate packing and unpacking.
With 2^24 available packet numbers and assuming a packet update rate of 5Hz, the counter would overflow 932 hours into the mission. More than enough to deplete our battery.

|  Addresses | Content  |
|----------|---|
| 0x03 - 0x06  | Time since boot - LSB to MSB |

We also need a high resolution time reference to be able to locate important mission events in context. This allows us to say, for instance, that our balloon burst at exactly T+49:22.43, instead of only having packet 32810 as reference. 
We allocated 32 bits to this function. Since we chose milliseconds as our unit, the counter would overflow 1193 hours into the mission, again, more than enough.

|  Addresses | Content  |
|----------|---|
| 0x07 - 0x08  | BMP temperature - LSB to MSB |

Having an accurate temperature measurement is also very important for our mission since it is a key component of our altitude sensing: the formula for calculating altitude based on atmospheric pressure depends on it. We'll use Bosch's BMP180 sensor, which has a double utility: it measures both temperature and pressure. We'll pull the data according to our update rate, apply the calibration coefficients stored in the chip's EEPROM to obtain true temperature and send it, LSB-first, as a 16-bit signed integer that represents 1/10 of a degree C. This variable would overflow at ±3276.8 °C, but if we reach those temperature, the overflow here would be the minor of our concerns. Especially since temperatures below -273.15 °C are physically impossible. Oh well.

