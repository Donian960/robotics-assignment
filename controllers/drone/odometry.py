class Odometry:
  
  x = 0
  y = 0
  theta = 0
  
  left_sensor_previous = 0
  right_sensor_previous = 0
  
  def __init__(self, x, y, theta, left_sensor, right_sensor):
      self.x = x
      self.y = y
      self.theta = theta
      self.left_sensor_previous = left_sensor
      self.right_sensor_previous = right_sensor
  
  def step(self, left_sensor_value, right_sensor_value):
      dL = left_sensor_value - self.left_sensor_previous
      dR = right_sensor_value - self.right_sensor_previous
      dS = (dL + dR) / 2
      
      
      
      
      self.left_sensor_previous = left_sensor_value
      self.right_sensor_previous = right_sensor_value