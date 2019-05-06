#!/usr/bin/env python3
"""
Script to drive a donkey 2 car using the RC controller instead of the web controller

Usage:
    manage.py (drive)
    manage.py (calibrate)

Options:
    -h --help        Show this screen.
"""

from docopt import docopt
import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle, RCReceiver
from donkeycar.parts.datastore import TubGroup, TubWriter, TubHandler
from donkeycar.parts.clock import Timestamp
from donkeycar.parts.transform import Lambda


def drive(cfg):

    """
    Construct a working robotic vehicle from many parts. Each part runs as a job
    in the Vehicle loop, calling either its run or run_threaded method depending
    on the constructor flag `threaded`. All parts are updated one after another
    at the framerate given in cfg.DRIVE_LOOP_HZ assuming each part finishes
    processing in a timely manner. Parts may have named outputs and inputs. The
    framework handles passing named outputs to parts requesting the same named
    input.
    """

    donkey_car = dk.vehicle.Vehicle()

    clock = Timestamp()
    donkey_car.add(clock, outputs=['timestamp'])

    cam = PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    donkey_car.add(cam, outputs=['cam/image_array'], threaded=True)

    # create the RC receiver
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    donkey_car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    donkey_car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])

    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM) 

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    donkey_car.add(steering, inputs=['user/angle'])
    donkey_car.add(throttle, inputs=['user/throttle'])

    # add tub to save data
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'timestamp']
    types = ['image_array', 'float', 'float', 'str']

    # multiple tubs
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)

    # single tub
    # tub = TubWriter(path=cfg.TUB_PATH, inputs=inputs, types=types)
    donkey_car.add(tub, inputs=inputs, run_condition='user/throttle_on')

    # run the vehicle
    donkey_car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg):
    """
    Construct an auxiliary robotic vehicle from only the RC controllers and
    prints their values. The RC remote usually has a tuning pot for the throttle
    and steering channel. In this loop we run the controllers and simply print
    their values in order to allow centering the RC pwm signals.
    """
    donkey_car = dk.vehicle.Vehicle()

    clock = Timestamp()
    donkey_car.add(clock, outputs=['timestamp'])

    # create the RC receiver
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    donkey_car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    donkey_car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])

    # create the lambda function for plotting
    def plotter(angle, steering_on, throttle, throttle_on):
        print('angle=%+5.4f, steering_on=%1d, throttle=%+5.4f, throttle_on=%1d' %
              (angle, steering_on, throttle, throttle_on))

    plotter_part = Lambda(plotter)
    # add plotter part
    donkey_car.add(plotter_part, inputs=['user/angle', 'user/steering_on',
                                         'user/throttle', 'user/throttle_on'])

    # run the vehicle
    donkey_car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    if args['drive']:
        drive(cfg)
    elif args['calibrate']:
        calibrate(cfg)




