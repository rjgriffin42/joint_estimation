#!/usr/bin/env python
import time
import rospy
from joint_estimation.msg import AxesValues
from Adafruit_I2C import Adafruit_I2C

# Mag 3310 Driver Class
class Mag_Driver():

  def __init__(self):
    # Get the ~private namespace parameters from command line or launch file
    rate = float(rospy.get_param('~rate', '10.0'))
    address = rospy.get_param('~address', '0x0e')
    no_sensors = rospy.get_param('~no_sensors', 1)

    # Create message object
    address = 0x0e
    DEVICE_REG = 1
    i2c = Adafruit_I2C(address)

    # Create a publisher for magnet messages
    pub = rospy.Publisher("magnet_topic", AxesValues, queue_size = 0)
    
    # Initialize messages
    msg = AxesValues()
    for i in range(0, no_sensors):
      msg.id.append(i)
      msg.x_axis.append(0)
      msg.y_axis.append(0)
      msg.z_axis.append(0)

    # Main while loop
    while not rospy.is_shutdown():
      #initialize bus
      i2c.write8(17,128)
      i2c.write8(16,1)
      data = [0, 0, 0, 0, 0, 0]

      time.sleep(0.0002) 
  
      #read x
      i2c.write8(1,0)
      time.sleep(0.0002)
      data[0] = i2c.readU8(1)
      time.sleep(0.0002)
      i2c.write8(2,0)
      time.sleep(0.0002)
      data[1] = i2c.readU8(2)
      msg.x_axis[0] = data[1] + (data[0] << 8)

      i2c.write8(3,0)
      time.sleep(0.0002)
      data[2] = i2c.readU8(3)
      time.sleep(0.0002)
      i2c.write8(4,0)
      time.sleep(0.0002)
      data[3] = i2c.readU8(4)
      msg.y_axis[0] = data[3] + (data[2] << 8)

      i2c.write8(5,0)
      time.sleep(0.0002)
      data[4] = i2c.readU8(5)
      time.sleep(0.0002)
      i2c.write8(6,0)
      time.sleep(0.0002)
      data[5] = i2c.readU8(6)
      msg.z_axis[0] = data[5] + (data[4] << 8)

      msg.id[0] = 0
      
      # publish data out
      pub.publish(msg)
     
      # Sleep for a while before publishing new messages
      if rate:
        rospy.sleep(1/rate)
      else:
        rospy.sleep(1.0)

# Main function
if __name__ == '__main__':
  rospy.init_node('mag3110_driver')
  # Go to class functions that do all the heavy lifting. Do error checking.
  md = Mag_Driver()  
  try:
    md = Mag_Driver()  
  except rospy.ROSInterruptException: pass
