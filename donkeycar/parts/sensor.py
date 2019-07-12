"""
sensor.py
Classes for sensory information.

"""


class Odometer:
    """
    Odometric part to measure the speed usually through hall sensors sensing
    magnets attached to the drive system.
    """
    def __init__(self, gpio=6, tick_per_meter=76, weight=0.5, debug=False):
        """
        :param gpio: gpio of sensor being connected
        :param tick_per_meter: how many signals per meter
        :param weight: weighting of current measurement in average speed
                        calculation
        """
        import pigpio
        self._gpio = gpio
        self._tick_per_meter = tick_per_meter
        self._pi = pigpio.pi()
        self._last_tick = None
        self._last_tick_speed = None
        self._weight = weight
        self._avg = 0.0
        self._max_speed = 0.0
        self._distance = 0
        self._debug = debug

        # pigpio callback mechanics
        self._pi.set_pull_up_down(self._gpio, pigpio.PUD_UP)
        self._cb = self._pi.callback(self._gpio, pigpio.EITHER_EDGE, self._cbf)
        print("Odometer added at gpio {}".format(gpio))

    def _cbf(self, gpio, level, tick):
        """ Callback function for pigpio interrupt gpio. Signature is determined
        by pigpiod library. This function is called every time the gpio changes
        state as we specified EITHER_EDGE.
        :param gpio: gpio to listen for state changes
        :param level: rising/falling edge
        :param tick: # of mu s since boot, 32 bit int
        """
        import pigpio
        if self._last_tick is not None:
            diff = pigpio.tickDiff(self._last_tick, tick)
            self._avg = self._weight * diff + (1.0 - self._weight) * self._avg
            self._distance += 1
        self._last_tick = tick

    def run(self):
        """
        Knowing the tick time in mu s and the ticks/m we calculate the speed. If
        ticks haven't been update since the last call we assume speed is zero
        :return speed: in m / s
        """
        speed = 0.0
        if self._last_tick_speed != self._last_tick and self._avg != 0.0:
            speed = 1.0e6 / (self._avg * self._tick_per_meter)
            self._max_speed = max(self._max_speed, speed)
        self._last_tick_speed = self._last_tick
        if self._debug:
            print("Speed =", speed)
        return speed

    def shutdown(self):
        """
        Donkey parts interface
        """
        import pigpio
        self._cb.cancel()
        print('Maximum speed {0:4.2f}, total distance {1:4.2f}'
              .format(self._max_speed,
                      float(self._distance) / float(self._tick_per_meter)))

