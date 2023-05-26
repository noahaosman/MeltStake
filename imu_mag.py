#!/usr/bin/python3

from math import sin, cos, asin, atan2, sqrt, pi

from icm20602 import ICM20602
from mmc5983 import MMC5983


class ImuMag:

    def __init__(self):

        self.icm = ICM20602()
        self.mmc = MMC5983(i2cbus=None)

    def get_imu(self):

        data = self.icm.read_all()
        acc = [data.a.x, data.a.y, data.a.z]
        # gyr = [data.g.x, data.g.y, data.g.z]

        return acc

    def get_mag(self):

        # read the data
        data = self.mmc.read_data()
        mag = [data.x, data.y, data.z]
        # mag_raw = [data.x_raw, data.y_raw, data.z_raw]
        # print(mag_raw)
        # print(mag)

        return mag

    def cal_mag(self):

        self.mmc.calibrate()
        # cal = [self.mmc.caldata[0], self.mmc.caldata[1], self.mmc.caldata[2]]
        # print(cal)

    def process_imu(self, accRaw):
        """ Processes raw IMU data to derive pitch and roll.
        Pitch is positive when the bow lifts up.
        Roll is positive when the port side lifts up.
        """

        # desired XYZ directions (standard)
        # X - point toward bow
        # Y - point to port
        # Z - point up

        # uncomment the relevant lines depending on the IMU orientation
        accRaw[0] = -accRaw[0]  # imu x axis points to the stern
        accRaw[1] = -accRaw[1]  # imu y axis points to the starboard
        accRaw[2] = -accRaw[2]  # imu z axis points down
        try:
            # normalize the raw accelerometer data
            norm_factor = sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2])
            accXnorm = accRaw[0] / norm_factor
            accYnorm = accRaw[1] / norm_factor
            pitch = asin(accXnorm)
            roll = -asin(accYnorm/cos(pitch))
            # convert from radians to degrees
            pitch_deg = pitch * (180/pi)
            roll_deg = roll * (180/pi)
        except Exception as e:
            pitch_deg = 0
            roll_deg = 0
        return pitch_deg, roll_deg

    def process_mag(self, magRaw, pitch_deg, roll_deg):
        """ Processes raw magnetometer data to derive heading.
        Compensates for pitch and roll.
        Heading is measured clockwise in degrees from magnetic north.
        Pitch and roll are expected to be in degrees.
        *** This method is not outputting correct heading values. ***
        """

        # desired XYZ directions (standard)
        # X - point toward bow
        # Y - point to port
        # Z - point up

        # uncomment the relevant lines depending on the magnetometer orientation
        # magRaw[0] = -magRaw[0]  # mag x axis points to the stern
        magRaw[1] = -magRaw[1]  # mag y axis points to the starboard
        # magRaw[2] = -magRaw[2]  # mag z axis points down

        # convert from degrees to radians
        pitch = pitch_deg * (pi/180)
        roll = roll_deg * (pi/180)

        # compensate for the pitch and roll
        magXcomp = magRaw[0]*cos(pitch) + magRaw[2]*sin(pitch)
        magYcomp = magRaw[0]*sin(roll)*sin(pitch) + magRaw[1]*cos(roll) - magRaw[2]*sin(roll)*cos(pitch)
        # magXcomp = magRaw[0]*cos(pitch) - magRaw[2]*sin(pitch)
        # magYcomp = magRaw[0]*sin(roll)*sin(pitch) + magRaw[1]*cos(roll) + magRaw[2]*sin(roll)*cos(pitch)

        # compensated heading in radians
        heading = atan2(magYcomp, magXcomp)

        # convert from radians to degrees
        heading_deg = heading * (180/pi)

        # convert heading from math angles (degrees CCW from x) to azimuth (degrees CW from north)
        # heading_deg = 90 - heading_deg

        # keep heading in the range 0 - 360
        if heading_deg < 0:
            heading_deg += 360

        return heading_deg

    def main(self):

        self.cal_mag()
        acc = self.get_imu()
        mag = self.get_mag()
        # print(acc)
        # print(mag)

        pitch, roll = self.process_imu(acc)
        heading = self.process_mag(mag, pitch, roll)
        # print(heading)

        return pitch, roll, heading
