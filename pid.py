import time

MANUAL = 0
AUTOMATIC = 1
 
DIRECT = 0
REVERSE = 1

class pid():
    def __init__(self, kp, ki, kd, cycletime = 100):
        # working variables
        self.lastTime = 0
        self.Output = 0
        self.Setpoint = 0
        self.ITerm = 0
        self.lastInput = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.SampleTime = cycletime
        self.outMin = 0
        self.outMax = 1
        self.inAuto = False
        self.controllerDirection = DIRECT
        
    def SetTarget(self, target):
        self.Setpoint = target
        # print("PID target", self.Setpoint)
        
    def GetOutput(self):
        return self.Output
    
    def Compute(self, Input):
        if not self.inAuto:
            # print("Not in auto mode")
            return 0
        now = time.ticks_ms()
        timeChange = (now - self.lastTime)
        if timeChange >= self.SampleTime:
            # print("PID Input", Input)
            # Compute all the working error variables
            error = self.Setpoint - Input
            self.ITerm += (self.ki * error)
            if self.ITerm > self.outMax:
                self.ITerm= self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm= self.outMin
            dInput = (Input - self.lastInput)
     
            # Compute PID Output
            self.Output = self.kp * error + self.ITerm - self.kd * dInput
            if self.Output > self.outMax:
                # print("Clipped to max", self.Output, self.outMax)
                self.Output = self.outMax
            elif self.Output < self.outMin:
                # print("Clipped to min", self.Output, self.outMin)
                self.Output = self.outMin
            # print("PID output", self.Output)
     
            # Remember some variables for next time
            self.lastInput = Input
            self.lastTime = now
     
    def SetTunings(self, Kp, Ki, Kd):
        if (Kp<0 or Ki<0 or Kd<0): return

        SampleTimeInSec = SampleTime * 0.001
        self.kp = Kp
        self.ki = Ki * SampleTimeInSec
        self.kd = Kd / SampleTimeInSec
     
        if controllerDirection == REVERSE:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd
            
    def SetSampleTime(self, NewSampleTime):
       if (NewSampleTime > 0):
        ratio  = NewSampleTime / SampleTime
        self.ki *= ratio
        self.kd /= ratio
        self.SampleTime = NewSampleTime
     
    def SetOutputLimits(self, Min, Max):
        if Min > Max: return
        self.outMin = Min
        self.outMax = Max

        if self.Output > self.outMax:
            self.Output = self.outMax
        elif self.Output < self.outMin:
            self.Output = self.outMin

        if self.ITerm > self.outMax:
            self.ITerm= self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm= self.outMin
     
    def SetMode(self, Mode):
        newAuto = (Mode == AUTOMATIC)
        if newAuto != self.inAuto:
            # we just went from manual to auto
            self.Initialize()
        self.inAuto = newAuto
     
    def Initialize(self):
        self.lastInput = 0
        self.ITerm = self.Output
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin
     
    def SetControllerDirection(self, Direction):
        controllerDirection = Direction
