from machine import Pin, time_pulse_us  # pyright: ignore[reportMissingModuleSource]
from asyncio import sleep_ms, create_task  # pyright: ignore[reportGeneralTypeIssues]


class HCSR04:
    """
    Driver to use the untrasonic sensor HC-SR04.
    The sensor range is between 2cm and 4m.
    The timeouts received listening to echo pin are converted to OSError('Out of range')
    """

    cms = 0
    mm = 0

    # echo_timeout_us is based in chip range limit (400cm)
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=500 * 2 * 30):
        """
        trigger_pin: Output pin to send pulses
        echo_pin: Readonly pin to measure the distance. The pin should be protected with 1k resistor
        echo_timeout_us: Timeout in microseconds to listen to echo pin.
        By default is based in sensor limit range (4m)
        """
        self.echo_timeout_us = echo_timeout_us
        # Init trigger pin (out)
        self.trigger = Pin(trigger_pin, mode=Pin.OUT)
        self.trigger.value(0)

        # Init echo pin (in)
        self.echo = Pin(echo_pin, mode=Pin.IN)

    def start(self):
        create_task(self.distance_cm())
        create_task(self.distance_mm())

    async def _send_pulse_and_wait(self):
        """
        Send the pulse to trigger and listen on echo pin.
        We use the method `machine.time_pulse_us()` to get the microseconds until the echo is received.
        """
        self.trigger.value(0)  # Stabilize the sensor
        await sleep_ms(5 / 1000)
        self.trigger.value(1)
        # Send a 10us pulse.
        await sleep_ms(10 / 1000)
        self.trigger.value(0)
        try:
            pulse_time = time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110:  # 110 = ETIMEDOUT
                raise OSError("Out of range")
            raise ex

    async def distance_mm(self):
        """
        Get the distance in milimeters without floating point operations.
        """
        pulse_time = await self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.34320 mm/us that is 1mm each 2.91us
        # pulse_time // 2 // 2.91 -> pulse_time // 5.82 -> pulse_time * 100 // 582
        self.mm = pulse_time * 100 // 582
        return self.mm

    async def distance_cm(self):
        """
        Get the distance in centimeters with floating point operations.
        It returns a float
        """
        pulse_time = await self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.034320 cm/us that is 1cm each 29.1us
        self.cms = (pulse_time / 2) / 29.1
        return self.cms

    def detectWallsCms(self, cms):
        if cms <= self.cms:
            return True
        return False

    def detectWallsMm(self, mm):
        if mm <= self.mm:
            return True
        return False
