"""
sensor.py
Classes for sensory information.

"""

import pigpio

class Odometer:
    """
    Odometric part to measure the speed usually through hall sensors sensing
    magnets attached to the drive system.
    """
    def __init__(self, gpio=6, tick_per_meter=76, weight=0.5):
        self._gpio = gpio
        self._tick_per_meter = tick_per_meter
        self._pi = pigpio.pi()
        self._last_tick = None
        self._last_tick_speed = None
        self._weight = weight
        self._avg = 0

        # pigpio callback mechanics
        self._pi.set_pull_up_down(self._gpio, pigpio.PUD_UP)
        self.cb = self._pi.callback(self._gpio, pigpio.EITHER_EDGE, self.cbf)
        print("Odometer added at gpio {}".format(gpio))

    def cbf(self, gpio, level, tick):

        """ Callback function for pigpio interrupt gpio. Signature is determined
        by pigpiod library. This function is called every time the gpio changes
        state as we specified EITHER_EDGE """
        if self._last_tick is not None:
            diff = pigpio.tickDiff(self._last_tick, tick)
            self._avg = self._weight * diff + (1.0 - self._weight) * self._avg
            print("l={0} diff={1:5.2f} avg={2:5.2f}".format(level, diff / 1000, self._avg / 1000))
        self._last_tick = tick

    def run(self):
        """
        Knowing the tick time in ms and the ticks/m we calculate speed where ticks
        are measured in ms. If ticks haven't been update since the last call we
        assume speed is zero
        :return speed: in m / s
        """
        speed = 0.0
        print("self._last_tick_speed={} self._last_tick={} self._avg={}"
              .format(self._last_tick_speed, self._last_tick, self._avg))
        if self._last_tick_speed != self._last_tick and self._avg != 0.0:
            self._last_tick_speed = self._last_tick
            speed = 1000 / (self._avg * self._tick_per_meter)
        print("speed={0:3.2f}".format(speed))
        return speed

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.cb.cancel()

