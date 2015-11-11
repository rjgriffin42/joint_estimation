#!/usr/bin/env python
import rospy
from joint_estimation.msg import AxesValues
from Adafruit_I2C import Adafruit_I2C

# Mag 3310 Driver Class
class Mag_Driver():

  def __init__(self):
    # Get the ~private namespace parameters from command line or launch file
    rate = float(rospy.get_param('~rate', '10.0'))
    address = rospy.get_param('~address', '0x60')
    no_sensors = rospy.get_param('~no_sensors', 1)

    # Create message object
    address = 0x60
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
      i2c.write8(DEVICE_REG, 0x01) #FIXME is this needed?

      # read in six bytes of message FIXME have this iterate of the different nodes
      data = []
      # Make list of 6 bytes
      for i in range (0, 6):
        data.append(i2c.readU8(DEVICE_REG))

      msg.id[0] = 0
      msg.x_axis[0] = (data[0] << 8) + data[1]
      msg.y_axis[0] = (data[2] << 8) + data[3]
      msg.z_axis[0] = (data[4] << 8) + data[5]
      
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
  print "calling main"
  # Go to class functions that do all the heavy lifting. Do error checking.
  md = Mag_Driver()  
  try:
    md = Mag_Driver()  
  except rospy.ROSInterruptException: pass
