"""
sensor.py
Classes for sensory information.

"""
from prettytable import PrettyTable


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
        :param debug: if debug info should be printed
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
        self._run_counter = 0

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
        if self._debug and self._run_counter % 20 == 0:
            print("Speed =", speed)
        self._run_counter += 1
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
        if self._debug:
            print('Total num ticks {}'.format(self._max_speed, self._distance))


class LapTimer:
    """
    LapTimer part to count the number of laps, and lap times
    """
    def __init__(self, gpio=16, min_time=1.0):
        """
        :param gpio: gpio of sensor being connected
        :param debug: if debug info should be printed
        """
        import pigpio
        self.gpio = gpio
        self.pi = pigpio.pi()
        self.last_tick = None
        self.lap_count = 0
        self.lap_times = []
        self.debug = debug
        self.min_time = min_time  # minimum lap time

        # pigpio callback mechanics
        self.pi.set_pull_up_down(self.gpio, pigpio.PUD_OFF)
        self.cb = self.pi.callback(self.gpio, pigpio.FALLING_EDGE, self.cbf)
        print("LapTimer added at gpio {}".format(gpio))

    def cbf(self, gpio, level, tick):
        """ Callback function for pigpio interrupt gpio. Signature is determined
        by pigpiod library. This function is called every time the gpio changes
        from high to low which will happen for the IR sensor TSOP4838 (and
        others)
        :param gpio: gpio to listen for state changes
        :param level: rising/falling edge
        :param tick: # of mu s since boot, 32 bit int
        """
        import pigpio
        if self.last_tick is not None:
            diff = pigpio.tickDiff(self.last_tick, tick)
            sec = diff * 1.0e-6
            if sec > self.min_time:
                self.lap_times.append(sec)
                print('Lap {} completed in {}s'.format(self.lap_count, sec))
                self.lap_count += 1
        self.last_tick = tick


    def run(self):
        """
        :return: current lap number
        """
        return self.lap_count

    def shutdown(self, exit_info=None):
        """
        Donkey parts interface
        """
        import pigpio
        self.cb.cancel()
        print("Lap Summary: (times in s)")
        pt = PrettyTable()
        pt.field_names = ['Lap', 'Time']
        info_list = []
        for i, t in enumerate(self.lap_times[1:]):
            info_list.append([i, t])
            pt.add_row([i, '{0:6.3f}'.format(t)])
        exit_info['LapTimer'] = info_list
        print(pt)


