import asyncio
from math import pi

from machine import PWM, Pin
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
        return int(
            (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        )  # pyright: ignore[reportGeneralTypeIssues]

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

    async def deinit(self):
        """deinit PWM Pins"""
        self.stop()
        await asyncio.sleep(0.1)
        self.in1.deinit()
        self.in2.deinit()
        self.in3.deinit()
        self.in4.deinit()


# https://github.com/peterhinch/micropython-async/blob/master/v3/primitives/encoder.py
# encoder.py Asynchronous driver for incremental quadrature encoder.

# Copyright (c) 2021-2022 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file

# Thanks are due to @ilium007 for identifying the issue of tracking detents,
# https://github.com/peterhinch/micropython-async/issues/82.
# Also to Mike Teachman (@miketeachman) for design discussions and testing
# against a state table design
# https://github.com/miketeachman/micropython-rotary/blob/master/rotary.py

# motor data: https://robokits.co.in/motors/n20-metal-gear-micro-motors/n20-metal-gear-encoder-motor/ga12-n20-12v-1000-rpm-all-metal-gear-micro-dc-encoder-motor-with-precious-metal-brush
WHEEL_DIA = const(0.43)  # in cms
CPR = const(28)
GEAR_RATIO = const(30)


class Encoder:
    rpm2cms = lambda self, wheel_diameter=WHEEL_DIA: self.value() * wheel_diameter * pi

    def __init__(
        self,
        pin_x,
        pin_y,
        v=0,
        div=CPR * GEAR_RATIO,
        vmin=None,
        vmax=None,  # pyright: ignore[reportGeneralTypeIssues]
        mod=None,
        callback=lambda a, b: None,
        args=(),
        delay=100,
    ):
        self._pin_x = pin_x
        self._pin_y = pin_y
        self._x = pin_x()
        self._y = pin_y()
        self._v = v * div  # Initialise hardware value
        self._cv = v  # Current (divided) value
        self.delay = delay  # Pause (ms) for motion to stop/limit callback frequency

        if ((vmin is not None) and v < vmin) or ((vmax is not None) and v > vmax):
            raise ValueError("Incompatible args: must have vmin <= v <= vmax")
        self._tsf = asyncio.ThreadSafeFlag()  # pyright: ignore[reportGeneralTypeIssues]
        trig = Pin.IRQ_RISING | Pin.IRQ_FALLING
        try:
            xirq = pin_x.irq(trigger=trig, handler=self._x_cb, hard=True)
            yirq = pin_y.irq(trigger=trig, handler=self._y_cb, hard=True)
        except TypeError:  # hard arg is unsupported on some hosts
            xirq = pin_x.irq(trigger=trig, handler=self._x_cb)
            yirq = pin_y.irq(trigger=trig, handler=self._y_cb)
        asyncio.create_task(self._run(vmin, vmax, div, mod, callback, args))

    # Hardware IRQ's. Duration 36μs on Pyboard 1 ~50μs on ESP32.
    # IRQ latency: 2nd edge may have occured by the time ISR runs, in
    # which case there is no movement.
    def _x_cb(self, pin_x):
        if (x := pin_x()) != self._x:
            self._x = x
            self._v += 1 if x ^ self._pin_y() else -1
            self._tsf.set()

    def _y_cb(self, pin_y):
        if (y := pin_y()) != self._y:
            self._y = y
            self._v -= 1 if y ^ self._pin_x() else -1
            self._tsf.set()

    async def _run(self, vmin, vmax, div, mod, cb, args):
        pv = self._v  # Prior hardware value
        pcv = self._cv  # Prior divided value passed to callback
        lcv = pcv  # Current value after limits applied
        plcv = pcv  # Previous value after limits applied
        delay = self.delay
        while True:
            await self._tsf.wait()
            await asyncio.sleep_ms(
                delay
            )  # Wait for motion to stop. # pyright: ignore[reportGeneralTypeIssues]
            hv = self._v  # Sample hardware (atomic read).
            if hv == pv:  # A change happened but was negated before
                continue  # this got scheduled. Nothing to do.
            pv = hv
            cv = round(hv / div)  # cv is divided value.
            if not (dv := cv - pcv):  # dv is change in divided value.
                continue  # No change
            lcv += dv  # lcv: divided value with limits/mod applied
            lcv = lcv if vmax is None else min(vmax, lcv)
            lcv = lcv if vmin is None else max(vmin, lcv)
            lcv = lcv if mod is None else lcv % mod
            self._cv = lcv  # update ._cv for .value() before CB.
            if lcv != plcv:
                cb(lcv, lcv - plcv, *args)  # Run user CB in uasyncio context
            pcv = cv
            plcv = lcv

    def value(self):
        return self._cv
