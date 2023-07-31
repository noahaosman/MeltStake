#!/bin/bash

ARG1=${1:-'00'}


# Configure static IP
ip r | grep default
cat /etc/resolv.conf
IP_FILE='/etc/dhcpcd.conf'
read -r -d '' IP_LINE << EOM
interface wlan0
static ip_address=10.0.1.1$ARG1/24
static routers=10.0.1.1
static domain_name_servers=10.0.1.1
EOM
echo "$IP_LINE" >> "$IP_FILE"


# Setup RTC
wget https://raw.githubusercontent.com/raspberrypi/linux/rpi-4.14.y/arch/arm/boot/dts/overlays/i2c-rtc-gpio-overlay.dts
dtc -I dts -O dtb -o i2c-rtc-gpio.dtbo i2c-rtc-gpio-overlay.dts
cp i2c-rtc-gpio.dtbo /boot/overlays
rm i2c*
timedatectl set-timezone UTC

apt -y remove fake-hwclock
update-rc.d -f fake-hwclock remove
sed -i '/systemd/,+2 d' /lib/udev/hwclock-set
hwclock -r  # reads hwclock time
hwclock -w  # sets hwclock to the current system time
# on initial boot set to current time eg: sudo date -u -s '22 Jun 2023 18:34:00' 
# timedatectl shows system time and hwclock time


# Configure interfacing options
INTERFACE_FILE='/boot/config.txt'
read -r -d '' INTERFACE_LINE << EOM
dtoverlay=disable-bt
enable_uart=1
dtoverlay=spi1-3cs
dtparam=i2c_arm=on
dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3
dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=6,i2c_gpio_scl=7
dtoverlay=i2c-rtc-gpio,ds3231,i2c_gpio_sda=22,i2c_gpio_scl=23
EOM
echo "$INTERFACE_LINE" >> "$INTERFACE_FILE"
echo "i2c-dev" >> /etc/modules

# enable hardware serial port
raspi-config nonint do_serial 2

# enable i2c
raspi-config nonint do_i2c 0



# Configure wifi network settings
WPA_FILE='/etc/wpa_supplicant/wpa_supplicant.conf'
read -r -d '' WPA_LINE << EOM
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
  ssid="mixz"
  psk="chi10eps9"
  priority=3
}
network={
  ssid="ScienceShare"
  psk="SwimRobotSwim"
  priority=2
}
network={
  ssid="meltStake$ARG1"
  mode=2
  key_mgmt=WPA-PSK
  psk="raspberry"
  frequency=2437
  priority=1
}
EOM
echo "$WPA_LINE" > "$WPA_FILE"

# Install required Python packages
apt-get update
apt-get install -y python3-pip
apt-get install -y i2c-tools
apt-get install -y python3-dev
export PATH="/home/pi/.local/bin:$PATH"
apt-get install -y python3-numpy
apt-get install -y git
pip3 install Adafruit-Blinka
pip3 install adafruit-circuitpython-ads1x15
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-servokit
pip3 install adafruit-extended-bus
pip3 install spidev
apt-get install -y python3-pandas
pip3 install jinja2
pip3 install matplotlib
pip3 install pypdf2
apt-get install -y python3-smbus2

mkdir /home/pi/data
mkdir /home/pi/packages

# Install software for IMU/Magnetometer on Navigator
git clone https://github.com/bluerobotics/icm20602-python /home/pi/packages/icm20602-python
cd /home/pi/packages/icm20602-python
python3 setup.py install --user
git clone https://github.com/bluerobotics/mmc5983-python /home/pi/packages/mmc5983-python
cd /home/pi/packages/mmc5983-python
python3 setup.py install --user
git clone https://github.com/bluerobotics/llog-python /home/pi/packages/llog-python
cd /home/pi/packages/llog-python
python3 setup.py install --user

# Install Blue Robotics Bar30 pressure/temperature sensor software
git clone https://github.com/bluerobotics/ms5837-python /home/pi/packages/ms5837-python
cd /home/pi/packages/ms5837-python
python3 setup.py install --user

# Install Blue Robotics ping sonar software
git clone --single-branch --branch deployment https://github.com/bluerobotics/ping-python.git /home/pi/packages/ping-python
cd /home/pi/packages/ping-python
python3 setup.py install --user

# install camera code
git clone https://github.com/RoboticOceanographicSurfaceSampler/camera_capture.git
bash /home/pi/camera_capture/setup.sh
apt-get install -y mpv # this package lets you view video in terminal over ssh (bad quality!!) mpv --no-config --vo=tct <video file>

# configure main.py to run on boot
#echo 'python3 /home/pi/MeltStake/main.py -d $ARG1 -m deploy & 
#exit 0' >> /etc/rc.local




# to create sytstemctl things:
# create a file /etc/systemd/system/<____>.service
# populate with:
      # [Unit]
      # Description=<short description here>

      # [Service]
      # Type=simple
      # WorkingDirectory=/home/pi/<directory>
      # User=pi
      # StandardOutput=syslog
      # StandardError=syslog
      # SyslogIdentifier=<____>
      # ExecStart=/home/pi/<directory>/<____>.py
      # Restart=on-failure
      # RestartSec=<restart time>

      # [Install]
      # WantedBy=multi-user.target
# run sudo systemctl enable <____>