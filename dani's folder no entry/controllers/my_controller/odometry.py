import math

class Odometry:
  
  #Values attained through robot proto file
  DISTANCE_BETWEEN_WHEELS = 0.1054 #From translation of left wheel and right wheel
  WHEEL_DIAMETER = 0.042 #From 2 * Radius of left wheel
   
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
      #Use sensor values to work out distance travelled by each wheel
      dL = left_sensor_value - self.left_sensor_previous
      dR = right_sensor_value - self.right_sensor_previous
      
      #Derived by cancelling out pi in equation from the slides      
      dS_l = (dL * self.WHEEL_DIAMETER)/2
      dS_r = (dR * self.WHEEL_DIAMETER)/2

      #Calculate new position and angle using equations from the slides
      dS = (dS_l + dS_r) / 2
      dTheta = (dS_r - dS_l) / (2 * self.DISTANCE_BETWEEN_WHEELS)

      dX = dS * math.cos(self.theta + (dTheta/2))
      dY = dS * math.sin(self.theta + (dTheta/2))

      #Assign resulting values
      self.x += dX
      self.y += dY
      self.theta += dTheta
      
      self.left_sensor_previous = left_sensor_value
      self.right_sensor_previous = right_sensor_value