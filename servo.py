#Driver for servos

from machine import Pin, PWM

class Servo():
    """Control a single servo."""

    def __init__(self, aPin, aRange=180, aMinPulse=1000, aMaxPulse=2000, aPulseRate=50):
        """
        aPin = pin number
        aRange = servo angle range
        aMinPulse = pulse time (ns) for angle 0
        aMaxPulse = pulse time (ns) for max angle
        aPulseRate = pulse repitition rate (Hz), normally 50
        #Example:
        s1 = servo(2, 180, 800, 2200, 50)
        """
        self._pin = Pin(aPin, Pin.OUT)
        self._range = aRange
        self._minPulse = aMinPulse
        self._maxPulse = aMaxPulse
        self._pulseRate = aPulseRate
        self._pulseTime = int((self._minPulse + self._maxPulse) / 2)
        self._angle = aRange / 2
        self._pwm = PWM(self._pin, freq=self._pulseRate, duty_ns=self._pulseTime)
        #print("Created servo on pin", aPin)
        #print("Pulse rate:", self._pulseRate)
        #print("Pulse times:", self._minPulse, "to", self._maxPulse)

    @property
    def angle(self): return self._angle

    @angle.setter
    def angle( self, aValue ) :
        #print("Setting angle to",aValue)
        self.pulseTime = self.angle2pulse(aValue)
        self._angle = aValue
    
    @property
    def pulseTime(self): return self._pulseTime
    
    @pulseTime.setter
    def pulseTime(self, aValue):
        aValue = int(max(self._minPulse, aValue))
        aValue = min(self._maxPulse, aValue)
        #print("Servo pulse", aValue)
        #self._pwm.duty_ns(aValue)
        duty = int(65535 * float(aValue) / (1000000/self._pulseRate))
        self._pwm.duty_u16(duty)
        self._pulseTime = aValue
        #print("Pulse",self._pwm.duty_ns(), "at", self._pwm.freq(), "Hz")

    @staticmethod
    def map_between_ranges(v, inmin, inmax, outmin, outmax):
        return float(outmin+(float(v-inmin) / float(inmax-inmin)) * float(outmax-outmin))

    def angle2pulse(self, aValue):
        aValue = max(0, aValue)
        aValue = min(self._range, aValue)
        pulse = self.map_between_ranges(aValue, 0, self._range, self._minPulse, self._maxPulse)
        #print("Mapped", aValue, "to", pulse)
        return pulse
