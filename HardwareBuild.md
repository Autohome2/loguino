

This article describes how to build the loguino hardware.  Once built, it should look something like this:

<img><img src='http://farm6.static.flickr.com/5313/5896627429_80b40356ee.jpg' /></img>

# Shopping List #

You will need:

  * Arduino Mega2560 or compatible controller such as this one.
  * Adafruit [GPS Logger Shield](http://www.adafruit.com/products/98) or equivalent.
  * Sparkfun [LIS331 breakout](http://www.sparkfun.com/products/10345) or equivalent.
  * Arduino Mega [Protoshield Board](http://www.eztronics.nl/webshop/catalog/product_info.php/products_id/239)
  * RS232 to TTL converter:
    * MAX3232 Serial Convertor IC.
    * 5x0.1uf Capacitors
    * 2x5k resistors
  * Suitably large enclosure
  * RS232 Male Socket.

# Quickstart #

  * Wire the GPS logger shield so that the GPS is permenantly on (Pwr connects direct to ground.)
  * Wire the GPS to Serial2.
  * Wire the RS232 side of the RS232 converter to the RS232 Port
  * Wire the TTL side of the RS232 converter to Serial1
  * Wire the LIS331 in i2c configuration.

# Wiring the GPS Shield #

Build the GPS shield as per the [instructions](http://www.ladyada.net/make/gpsshield/solder.html).

Connect the internal pins as follows:

<img><img src='http://farm6.static.flickr.com/5268/5896609545_10a12c049d.jpg' /></img>

  * Connect the TX pin on the GPS shield to pin 17 on the protoshield.
  * Connect the RX pin on the GPS shield to pin 16 on the proto shield.
  * Connect the power pin to the ground pin on the GPS shield.
  * Connect LED 1 to pin 9
  * Connect LED 2 to pin 8

# Wiring the LIS331 Breakout #

The LIS331 breakout is mounted onto the proto Shield, you can choose to mount it either using header pins soldered to the proto shield, or by carefully drilling two holes in the shield and mounting using bolts and a spacer.

<img><img src='http://farm6.static.flickr.com/5119/5896652539_7ec0a7de40.jpg' /></img>

The pins on the LIS331 are wired as follows, all connections are made to the protoshield.

  * vcc -> 3.3v (Pin
  * cs -> 3.3v
  * sa0 ->3.3v
  * scl -> SCL Pin 21
  * sda -> SDA Pin 22
  * int1 -> Ground
  * int2 -> Ground
  * gnd-> Ground

## SA0 and Addressing ##
If the SA0 pin is held high (3.3v) the address of the LIS331 is set to 0b00011001. (0x19, 25) If the pin is held low (grounded) then the address of the LIS331 is set to 0b00011000. (0x18, 24)

The manual states: "The Slave ADdress (SAD) associated to the LIS331HH is 001100xb. SDO/SA0 pad can be used to modify less significant bit of the device address. If SA0 pad is connected to voltage supply, LSb is ‘1’ (address 0011001b) else if SA0 pad is connected to ground, LSb value is ‘0’ (address 0011000b). This solution permits to connect and address two different accelerometers to the same I2C lines."

# Wiring the MAX 3232 #

The MAX3232 is the trickiest part of the loguino to assemble, i used the DIP area on the protoshield, but if you are less confident you could use some stripboard, and mount that using some header pins.

The MAX3232 converts the TTL level (5v) signals from the serial output of the loguino, to the RS232 (12v) signals from the Megasquirt.  This is the diagram of the circuit:

<img><img src='http://farm7.static.flickr.com/6021/5897281044_71a4ecba95.jpg' /></img>

I used some stripboard initially then transferred on to the DIP Area once it was working correctly.

<img><img src='http://farm6.static.flickr.com/5156/5896738525_e9c436f7b0.jpg' /></img>
