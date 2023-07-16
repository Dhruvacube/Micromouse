from machine import Pin, PWM
import time
from micropython import const

MAX_DUTY_CYCLE = const((2**16) - 1)
MIN_DUTY_CYCLE = const(0)
MIN_SPEED = const(0)
MAX_SPEED = const(100)


class N20Motor:
    FREQUENCY = 20000

    def __init__(self, in1, in2, in3, in4, enp1=None, enp2=None, enp3=None, enp4=None):
        self.in1 = PWM(Pin(in1, Pin.OUT))
        self.in2 = PWM(Pin(in2, Pin.OUT))
        self.in3 = PWM(Pin(in3, Pin.OUT))
        self.in4 = PWM(Pin(in4, Pin.OUT))
        self.enp1 = Pin(enp1, Pin.IN, Pin.PULL_UP) if enp1 is not None else None
        self.enp2 = Pin(enp2, Pin.IN, Pin.PULL_UP) if enp2 is not None else None
        self.enp3 = Pin(enp3, Pin.IN, Pin.PULL_UP) if enp3 is not None else None
        self.enp4 = Pin(enp4, Pin.IN, Pin.PULL_UP) if enp4 is not None else None
        self.in1.freq(self.FREQUENCY)
        self.in2.freq(self.FREQUENCY)
        self.in3.freq(self.FREQUENCY)
        self.in4.freq(self.FREQUENCY)

    def _speed(
        self,
        x,
        in_min=MIN_SPEED,
        in_max=MAX_SPEED,
        out_min=MIN_DUTY_CYCLE,
        out_max=MAX_DUTY_CYCLE,
    ):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def bakward(self, speed=MAX_SPEED):
        speed = self._speed(speed, MIN_SPEED, MAX_SPEED, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)
        self.in1.duty_u16(speed)
        self.in2.duty_u16(MIN_DUTY_CYCLE)
        self.in3.duty_u16(speed)
        self.in4.duty_u16(MIN_DUTY_CYCLE)

    def forward(self, speed=MAX_SPEED):
        speed = self._speed(speed, MIN_SPEED, MAX_SPEED, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)
        self.in1.duty_u16(MIN_DUTY_CYCLE)
        self.in2.duty_u16(speed)
        self.in3.duty_u16(MIN_DUTY_CYCLE)
        self.in4.duty_u16(speed)

    def left(self, speed=MAX_SPEED):
        speed = self._speed(speed, MIN_SPEED, MAX_SPEED, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)
        self.in1.duty_u16(speed)
        self.in2.duty_u16(MIN_DUTY_CYCLE)
        self.in3.duty_u16(MIN_DUTY_CYCLE)
        self.in4.duty_u16(speed)

    def right(self, speed=MAX_SPEED):
        speed = self._speed(speed, MIN_SPEED, MAX_SPEED, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)
        self.in1.duty_u16(MIN_DUTY_CYCLE)
        self.in2.duty_u16(speed)
        self.in3.duty_u16(speed)
        self.in4.duty_u16(MIN_DUTY_CYCLE)

    def stop(self):
        self.in1.duty_u16(MIN_DUTY_CYCLE)
        self.in2.duty_u16(MIN_DUTY_CYCLE)

        self.in3.duty_u16(MIN_DUTY_CYCLE)
        self.in4.duty_u16(MIN_DUTY_CYCLE)

    def deinit(self):
        """deinit PWM Pins"""
        self.stop()
        time.sleep(0.1)
        self.in1.deinit()
        self.in2.deinit()
        self.in3.deinit()
        self.in4.deinit()


class N20MotorEncoder:
    pass
