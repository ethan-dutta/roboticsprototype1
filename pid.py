import cv2 as cv
from PIL import Image
import imutils
from imutils.video import FPS
import time


class PIDController:
   def __init__(self, kP, kI, kD, setpoint):
       self.kP = kP
       self.kI = kI
       self.kD = kD


       self.setpoint = setpoint


       self.prevTime = time.time()


      
       self.previous_error = 0
       self.integral = 0


   def compute(self, process_variable, dt):
       error = self.setpoint - process_variable


       self.currTime = time.time()
       dt= self.currTime - self.prevTime
       self.prevTime = self.currTime


       # Proportional
       p_out = self.kP * error


       # Integral
       self.integral += error * dt
       i_out = self.kI * self.integral


       # Derivative
       derivative = (error - self.previous_error) / dt
       d_out = self.kD * derivative


       # Total PID output
       output = p_out + i_out + d_out


       self.previous_error = error


       return output




import matplotlib.pyplot as plt
import numpy as np


def main():
   cap = cv.VideoCapture(0, cv.CAP_V4L2)
   cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
   cap.set(cv.CAP_PROP_FRAME_HEIGHT, 320)
   cap.set(cv.CAP_PROP_FPS, 30)
   ret, frame = cap.read()


   first_gray =  cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
   refpoints = cv.goodFeaturesToTrack(
       cv.cvtColor(frame, cv.COLOR_BGR2GRAY),
       maxCorners = 100,
       qualityLevel=0.3,
       minDistance=7
   )


   current_gray = first_gray


   nextpoints, status, error = cv.calcOpticalFlowPyrLK(
       first_gray,
       current_gray,
       refpoints,
       None
   )




   setpoint = 0


   old = refpoints[status==1]
   new = nextpoints[status==1]


   pid = PIDController(
       kP=1.0,
       kI=0.1,
       kD=0.05,
       setpoint=setpoint
   )


   time = np.linspace(0, 10, 100)
   dt = time[1] - time[0]


   movement = np.median(old - new, axis=0)


   process_variable = movement[0]
   process_values = []


   for t in time:
       control_output = pid.compute(process_variable, dt)


       process_variable += (
           control_output * dt
           - 0.1 * (process_variable - 20) * dt
       )


       process_values.append(process_variable)




   plt.figure(figsize=(10, 6))


   plt.plot(
       time,
       process_values,
       label='Process Variable'
   )


   plt.axhline(
       y=setpoint,
       color='r',
       linestyle='--',
       label='Setpoint'
   )


   plt.xlabel('Time (s)')
   plt.ylabel('Temperature')
   plt.title('PID Controller Simulation')
   plt.legend()
   plt.grid()


   plt.show(block=False)
   plt.pause(0.1)


   cv.imshow("frame", frame)
   cv.waitKey(0)


   cap.release()
   cv.destroyAllWindows()
   plt.close()


main()
