"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk
import pigpio


class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, pulse)
        except OSError as err:
            print("Unexpected issue setting PWM (check wires to motor board): {0}".format(err))

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                 left_pulse=290, right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        pulse = dk.util.data.map_range(
            angle,
            self.LEFT_ANGLE, self.RIGHT_ANGLE,
            self.left_pulse, self.right_pulse
        )

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # set steering straight


class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self,
                 controller=None,
                 max_pulse=300,
                 min_pulse=490,
                 zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        # send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

    def run(self, throttle):
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                           0, self.MAX_THROTTLE,
                                           self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.util.data.map_range(throttle,
                                           self.MIN_THROTTLE, 0,
                                           self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # stop vehicle


class Adafruit_DCMotor_Hat:
    """
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    """
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0

    def run(self, speed):
        """
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        """
        if speed > 1 or speed < -1:
            raise ValueError("Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.util.data.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)

        self.motor.setSpeed(self.throttle)

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


class RCReceiver:
    """
    Class to read PWM from an RC control. Uses pigpio library.
    This is essentially a copy of (credit)
    http://abyz.me.uk/rpi/pigpio/code/read_PWM_py.zip. You will
    need a voltage divider from a 5V RC receiver to a 3.3V Pi input pin.

    A class to read PWM pulses and calculate their frequency
    and duty cycle.  The frequency is how often the pulse
    happens per second.  The duty cycle is the percentage of
    pulse high time per cycle.
    """
    MIN_OUT = -1
    MAX_OUT = 1

    def __init__(self, gpio, invert=False, jitter=0.015, no_action=None):
        """
        :param gpio: gpio pin connected to RC channel
        :param invert: invert value or run() within [MIN_OUT,MAX_OUT]
        :param jitter: threshold below which no signal is reported
        :param no_action: value within [MIN_OUT,MAX_OUT] if no RC signal is
                            sent. This is usually zero for throttle and steering
                            being the center values when the controls are not
                            pressed.
        """
        self.pi = pigpio.pi()
        self.gpio = gpio
        self._high_tick = None
        self._period = None
        self._high = None
        self._min_pwm = 1000
        self._max_pwm = 2000
        self._invert = invert
        self._jitter = jitter
        if no_action is not None:
            self._no_action = no_action
        else:
            self._no_action = (self.MAX_OUT - self.MIN_OUT) / 2.0

        self.pi.set_mode(self.gpio, pigpio.INPUT)
        self._cb = self.pi.callback(self.gpio, pigpio.EITHER_EDGE, self._cbf)

    def _update_param(self, tick):
        """ Helper function for callback function _cbf"""
        if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)
            return t

    def _cbf(self, gpio, level, tick):
        """ Callback function """
        if level == 1:
            self._period = self._update_param(tick)
            self._high_tick = tick
        elif level == 0:
            self._high = self._update_param(tick)

    def pulse_width(self):
        """
        Returns the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()

    def run(self):
        """
        Donkey parts interface, returns pulse mapped into [MIN_OUT,MAX_OUT] or [MAX_OUT,MIN_OUT]
        """
        # signal is a value in [0, (MAX_OUT-MIN_OUT)]
        signal = (self.pulse_width() - self._min_pwm) / (self._max_pwm - self._min_pwm) \
            * (self.MAX_OUT - self.MIN_OUT)
        # Assuming non-activity if the pulse is in the middle of [MIN_OUT,MAX_OUT]
        is_action = abs(signal - self._no_action) > self._jitter
        if not self._invert:
            return signal + self.MIN_OUT, is_action
        else:
            return -signal + self.MAX_OUT, is_action

    def shutdown(self):
        """
        Donkey parts interface
        """
        self.cancel()


class MockRCReceiver:
    """
    Mock of the above to run on host for debugging
    """
    def __init__(self):
        self.is_run_ = False

    def run(self):
        if self.is_run_ is True:
            self.is_run_ = False
            return 0.5, True
        else:
            self.is_run_ = True
            return 0.0, False

    def shutdown(self):
        pass

