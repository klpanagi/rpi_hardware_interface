Header header

# The thermal image aquired from flir_lepton
sensor_msgs/Image thermalImage

# The raw pixel values from flir_lepton
uint16[] rawValues

# The temperatures in pixels after calibration
std_msgs/Float32MultiArray temperatures
