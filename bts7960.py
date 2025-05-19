#Driver for the bts7960 43A high power motor controller.

from machine import Pin, PWM

class motor():
    """Control a motor connected to the bts7960 motor controller."""

    def __init__(self, aForward, aBackward, aFreq=10000, aPWM=65535):
        """aForward = tuple (On Pin #, PWM Pin #)
           aBackward = tuple (On Pin #, PWM Pin #)
           aFreq = pwm frequency.
           #Example:
           m1 = motor((19, 22), (21, 23), 5000)
        """
        self._on_forward = Pin(aForward[0], Pin.OUT)
        self._pwm_forward = PWM(Pin(aForward[1], Pin.OUT), freq=aFreq, duty_u16=0)
        self._on_backward = Pin(aBackward[0], Pin.OUT)
        self._pwm_backward = PWM(Pin(aBackward[1], Pin.OUT), freq=aFreq, duty_u16=0)
        self._speed = 0
        self._maxpwm = aPWM

    @staticmethod
    def setpin(aPin, aOn):
        """Set on/off (free wheeling) state of motor."""
        aPin.value(1 if aOn else 0)

    @property
    def speed(self): return self._speed

    @speed.setter
    def speed(self, aValue):
        """Set velocity and direction of motor with -100 <= aValue <= 100."""
        self._speed = aValue
        pos = True

        if aValue == 0 :
            # Free wheel mode?
            motor.setpin(self._on_backward, False)
            motor.setpin(self._on_forward, False)
            return
        motor.setpin(self._on_forward, True)
        motor.setpin(self._on_backward, True)

        if aValue < 0 :
            aValue = -aValue
            pos = False

        p = self.pc2pwm(min(100, aValue))
        # print("Motor duty", p)
        if pos:
            self._pwm_forward.duty_u16(p)
            self._pwm_backward.duty_u16(0)
        else:
            self._pwm_forward.duty_u16(0)
            self._pwm_backward.duty_u16(p)

    def pc2pwm(self, aPerc):
        """ Convert 0-100 range to 0-maxpwm (default 65535) range """
        return int((self._maxpwm * aPerc) // 100)
    