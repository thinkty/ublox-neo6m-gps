# U-blox NEO6M GPS Device Driver

A simple device driver for the U-blox NEO6M (might be compatible with other NEO-6 series although it is now deprecated according to the product [page](https://www.u-blox.com/en/product/neo-6-series)).
It uses [software-serial](https://github.com/thinkty/software-serial) to communicate with the GPS receiver over the Raspberry Pi GPIO pins. See the datasheet for the [module](https://content.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf) itself and the [NMEA-0183](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A119%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D) protocol for more information about the hardware.

## Software Requirements

Besides the actual hardware, to connect and use the GPS receiver with the Raspberry Pi (running Raspbian), it needs the following packages/executables.

- **kernel headers** : the kernel headers are needed to compile this kernel module. The version to download will depend on your (target) kernel version which can be found running `uname -r`.
- **dtc** : RPi uses [device tree](https://www.kernel.org/doc/Documentation/devicetree/usage-model.txt) for hardware enumeration. The `dtc` command will be used to compile the [overlay](https://www.raspberrypi.com/documentation/computers/configuration.html#part2) and it may already be installed by default. The compiled overlay can be placed in the overlays directory and configured to be applied on boot.

## Configuration

By default, the GPS receiver uses the *NMEA 0183* protocol with the following UART settings:
- Baudrate : 9600 bps
- Data bits : 8
- Parity bit : None
- Stop bit : 1
- Supported messages : [GSV](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A145%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [RMC](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A147%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [GSA](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A141%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [GGA](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A133%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [GLL](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A135%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [VTG](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A153%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D), [TXT](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf#%5B%7B%22num%22%3A151%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C841.89%2Cnull%5D)

Although the GPS receiver can handle other communication protocols and messages, this is the configuration used for this project.
For more information on the configuration, see the datasheet for the GPS receiver.

## Install
Clone the repository, run `make` to compile the device tree overlay and the kernel module.

### Device Tree Overlay
This kernel module uses the GPIO pins specified in [`overlay.dts`](https://github.com/thinkty/software-serial/blob/main/overlay.dts).
Therefore, the overlay must be compiled, and put into `/boot/firmware/overlays/` for it to be accessible.
Specify the overlay in `/boot/config.txt` to apply the overlay.
For example,
```
# Compile the overlay
dtc -@ -I dts -O dtb -o $(DT_OVERLAY).dtbo $(DT_OVERLAY).dts

# Place the overlay in the overlays dir
cp $(DT_OVERLAY).dtbo /boot/firmware/overlays/.

# Edit config.txt to include the overlay
echo "dtoverlay=$(DT_OVERLAY)" >> /boot/config.txt
```

After rebooting, check that the overlay has been properly applied to the device tree by running
```
dtc -I fs /sys/firmware/devicetree/base | less
```
There should be the module name in the device tree.

### Kernel Module
To install the kernel module, run :
```
insmod gps.ko
```

`sudo` may be needed due to permission.
`modprobe` may be used instead of `insmod` but there are no other dependencies for this module.
To ensure that the kernel module has been installed, check the messages from the kernel by running :
```
dmesg -wH
```

To remove (uninstall) the kernel module, run :
```
rmmod gps
```

## Usage

As it is a kernel module, you can use applications like `cat` and `echo` to read and write data to the device.
For example:

```
TODO:
```

## License
GPL
