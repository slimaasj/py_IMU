import smbus
bus = smbus.SMBus(1)
import time
import datetime
from numpy import array
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from transforms3d.euler import euler2quat as quaternion_from_euler

class PyIMUPub(Node):
    def __init__(self):
        super().__init__("pyIMUpub")

        pub_imu = self.create_publisher(
            msg_type=Imu,
            topic='imu',
            qos_profile=1,
        )
        
        RAD_TO_DEG = 57.29578
        M_PI = 3.14159265358979323846
        G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        AA =  0.40              # Complementary filter constant
        MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
        ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
        ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
        MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay

        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
        CFangleXFiltered = 0.0
        CFangleYFiltered = 0.0
        kalmanX = 0.0
        kalmanY = 0.0
        oldXMagRawValue = 0
        oldYMagRawValue = 0
        oldZMagRawValue = 0
        oldXAccRawValue = 0
        oldYAccRawValue = 0
        oldZAccRawValue = 0

        #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
        acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
        acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
        acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
        acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
        acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
        acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
        mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
        mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
        mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
        mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
        mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
        mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

        magXmin =  0
        magYmin =  0
        magZmin =  0
        magXmax =  0
        magYmax =  0
        magZmax =  0

        imu_msg = Imu()

        # TODO arrays not supported as parameter type ROS2
        imu_msg.orientation_covariance = [0.0025, 0.0, 0.0,
                                          0.0, 0.0025, 0.0,
                                          0.0, 0.0, 0.0025]
    
        imu_msg.angular_velocity_covariance = [0.002, 0.0, 0.0,
                                               0.0, 0.002, 0.0,
                                               0.0, 0.0, 0.002]
        
        imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                                  0.0, 0.04, 0.0,
                                                  0.0, 0.0, 0.04]

        imu_msg.header.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value

        publish_magnetometer = self.declare_parameter('publish_magnetometer', False).value

        def readACCx():
            acc_l = 0
            acc_h = 0

            acc_l = bus.read_byte_data(0x6A, 0x28)
            acc_h = bus.read_byte_data(0x6A, 0x29)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCy():
            acc_l = 0
            acc_h = 0
            
            acc_l = bus.read_byte_data(0x6A, 0x2A)
            acc_h = bus.read_byte_data(0x6A, 0x2B)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCz():
            acc_l = 0
            acc_h = 0
            
            acc_l = bus.read_byte_data(0x6A, 0x2C)
            acc_h = bus.read_byte_data(0x6A, 0x2D)

            acc_combined = (acc_l | acc_h <<8)
            return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readGYRx():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x22)
            gyr_h = bus.read_byte_data(0x6A, 0x23)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


        def readGYRy():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x24)
            gyr_h = bus.read_byte_data(0x6A, 0x25)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

        def readGYRz():
            gyr_l = 0
            gyr_h = 0
            
            gyr_l = bus.read_byte_data(0x6A, 0x26)
            gyr_h = bus.read_byte_data(0x6A, 0x27)

            gyr_combined = (gyr_l | gyr_h <<8)
            return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


        def readMAGx():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x28)
            mag_h = bus.read_byte_data(0x1C, 0x29)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        def readMAGy():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x2A)
            mag_h = bus.read_byte_data(0x1C, 0x2B)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        def readMAGz():
            mag_l = 0
            mag_h = 0
            
            mag_l = bus.read_byte_data(0x1C, 0x2C)
            mag_h = bus.read_byte_data(0x1C, 0x2D)

            mag_combined = (mag_l | mag_h <<8)
            return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        def initIMU():

            #initialise the accelerometer
            bus.write_byte_data(0x6A,0x10,0b10011111)         #ODR 3.33 kHz, +/- 8g , BW = 400hz
            bus.write_byte_data(0x6A,0x17,0b11001000)         #Low pass filter enabled, BW9, composite filter
            bus.write_byte_data(0x6A,0x12,0b01000100)         #Enable Block Data update, increment during multi byte read

            #initialise the gyroscope
            bus.write_byte_data(0x6A,0x11,0b10011100)         #ODR 3.3 kHz, 2000 dps

            #initialise the magnetometer
            bus.write_byte_data(0x1C,0x20, 0b11011100)        # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
            bus.write_byte_data(0x1C,0x21, 0b00100000)        # +/- 8 gauss
            bus.write_byte_data(0x1C,0x22, 0b00000000)        # Continuous-conversion mode

        initIMU()

        a = datetime.datetime.now()

        while True:

            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = readACCx()
            ACCy = readACCy()
            ACCz = readACCz()
            GYRx = readGYRx()
            GYRy = readGYRy()
            GYRz = readGYRz()
            MAGx = readMAGx()
            MAGy = readMAGy()
            MAGz = readMAGz()


            #Apply compass calibration
            MAGx -= (magXmin + magXmax) /2
            MAGy -= (magYmin + magYmax) /2
            MAGz -= (magZmin + magZmax) /2


            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            outputString = "Loop Time %5.2f " % ( LP )



            ###############################################
            #### Apply low pass filter ####
            ###############################################
            MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
            MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
            MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
            ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
            ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
            ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);

            oldXMagRawValue = MAGx
            oldYMagRawValue = MAGy
            oldZMagRawValue = MAGz
            oldXAccRawValue = ACCx
            oldYAccRawValue = ACCy
            oldZAccRawValue = ACCz

            #########################################
            #### Median filter for accelerometer ####
            #########################################
            # cycle the table
            for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
                acc_medianTable1X[x] = acc_medianTable1X[x-1]
                acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
                acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

            # Insert the lates values
            acc_medianTable1X[0] = ACCx
            acc_medianTable1Y[0] = ACCy
            acc_medianTable1Z[0] = ACCz

            # Copy the tables
            acc_medianTable2X = acc_medianTable1X[:]
            acc_medianTable2Y = acc_medianTable1Y[:]
            acc_medianTable2Z = acc_medianTable1Z[:]

            # Sort table 2
            acc_medianTable2X.sort()
            acc_medianTable2Y.sort()
            acc_medianTable2Z.sort()

            # The middle value is the value we are interested in
            ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
            ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
            ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];



            #########################################
            #### Median filter for magnetometer ####
            #########################################
            # cycle the table
            for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
                mag_medianTable1X[x] = mag_medianTable1X[x-1]
                mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
                mag_medianTable1Z[x] = mag_medianTable1Z[x-1]

            # Insert the latest values
            mag_medianTable1X[0] = MAGx
            mag_medianTable1Y[0] = MAGy
            mag_medianTable1Z[0] = MAGz

            # Copy the tablespy
            mag_medianTable2X = mag_medianTable1X[:]
            mag_medianTable2Y = mag_medianTable1Y[:]
            mag_medianTable2Z = mag_medianTable1Z[:]

            # Sort table 2
            mag_medianTable2X.sort()
            mag_medianTable2Y.sort()
            mag_medianTable2Z.sort()

            # The middle value is the value we are interested in
            MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
            MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
            MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];

            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN

            if 1:                       #Change to '0' to stop showing the angles from the accelerometer
                outputString += "#  ACCx %5.2f ACCy %5.2f ACCz %5.2f  #  " % (ACCx, ACCy, ACCz)

            if 1:                       #Change to '0' to stop  showing the angles from the gyro
                outputString +="\t# GRYx %5.2f  GYRy %5.2f  GYRz %5.2f # " % (GYRx, GYRy, GYRz)

            if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
                outputString +="\t# MAGx %5.2f  MAGy %5.2f  MAGz %5.2f # " % (MAGx, MAGy, MAGz)

            print(outputString)

            #ACCx, ACCy, ACCz
            #GYRx, GYRy, GYRz
            #MAGx, MAGy, MAGz

            imu_msg.linear_acceleration.x = float(ACCx)
            imu_msg.linear_acceleration.y = float(ACCy)
            imu_msg.linear_acceleration.z = float(ACCz)

            imu_msg.angular_velocity.x = float(GYRx)
            imu_msg.angular_velocity.y = float(GYRy)
            imu_msg.angular_velocity.z = float(GYRz)

            #mag_msg.magnetic_field.x = MAGx
            #mag_msg.magnetic_field.y = MAGy
            #mag_msg.magnetic_field.z = MAGz

            q = quaternion_from_euler(float(GYRx), float(GYRy), float(GYRz))
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            pub_imu.publish(imu_msg)

            #slow program down a bit, makes the output more readable
            time.sleep(0.03)


def main(args=None):
    rclpy.init(args=args)
    node = PyIMUPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()