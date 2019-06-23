#!/usr/bin/env python3
"""
Script to drive a donkey 2 car using the RC controller instead of the web
controller and to do a calibration of the RC throttle and steering triggers.

Usage:
    manage.py (drive) [--pid] [--no_cam]
    manage.py (calibrate)

Options:
    -h --help        Show this screen.
"""

from docopt import docopt
import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle, RCReceiver
from donkeycar.parts.datastore import TubHandler, TubWiper
from donkeycar.parts.clock import Timestamp
from donkeycar.parts.transform import Lambda, PIDController
from donkeycar.parts.sensor import Odometer


def drive(cfg, use_pid=False, no_cam=False, model_path=None):
    """
    Construct a working robotic vehicle from many parts. Each part runs as a job
    in the Vehicle loop, calling either its run or run_threaded method depending
    on the constructor flag `threaded`. All parts are updated one after another
    at the frame rate given in cfg.DRIVE_LOOP_HZ assuming each part finishes
    processing in a timely manner. Parts may have named outputs and inputs. The
    framework handles passing named outputs to parts requesting the same named
    input.
    """
    if no_cam:
        assert model_path is None, "Can't drive with pilot but w/o camera"

    donkey_car = dk.vehicle.Vehicle()

    clock = Timestamp()
    donkey_car.add(clock, outputs=['timestamp'])

    if not no_cam:
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        donkey_car.add(cam, outputs=['cam/image_array'], threaded=True)

    odo = Odometer()
    donkey_car.add(odo, outputs=['car/speed'])

    # drive by pid w/ speed or by throttle in [-1,1]?
    throttle_var = 'pilot/speed' if use_pid else 'pilot/throttle'
    # load model if present
    if model_path is not None:
        kl = dk.utils.get_model_by_type('linear', cfg)
        kl.load(model_path)
        outputs = ['pilot/angle', throttle_var]
        donkey_car.add(kl, inputs=['cam/image_array'], outputs=outputs)

    # create the RC receiver with 3 channels
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    rc_wiper = RCReceiver(cfg.DATA_WIPER_RC_GPIO, jitter=0.05, no_action=0)
    donkey_car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    donkey_car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])
    donkey_car.add(rc_wiper, outputs=['user/wiper', 'user/wiper_on'])

    # create the PWM steering and throttle controller for servo and esc
    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)
    donkey_car.add(steering, inputs=['user/angle'])

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    throttle_input = 'user/throttle' if model_path is None else 'pilot/throttle'
    if use_pid:
        class Rescaler:
            def run(self, controller_input):
                return controller_input * cfg.MAX_SPEED

        donkey_car.add(Rescaler(), inputs=['user/throttle'], outputs=['user/speed'])

        class PidError:
            def run(self, car_speed, user_speed):
                return car_speed - user_speed

        inputs = ['car/speed', 'user/speed' if model_path is None else 'pilot/speed']
        donkey_car.add(PidError(), inputs=inputs, outputs=['pid/error'])

        # add pid controller to convert throttle value into speed
        pid = PIDController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D, debug=False)
        donkey_car.add(pid, inputs=['pid/error'], outputs=[throttle_input])

    donkey_car.add(throttle, inputs=[throttle_input])

    # only record if cam is on and no auto-pilot
    if not no_cam and model_path is None:
        class RecordingCondition:
            def recording_condition(self, throttle_on, throttle_val):
                return throttle_on and throttle_val > 0

        donkey_car.add(RecordingCondition(),
                       inputs=['user/throttle_on', 'user/throttle'],
                       outputs=['user/recording'])

        # add tub to save data
        inputs = ['cam/image_array', 'user/angle', 'user/throttle',
                  'car/speed', 'timestamp']
        types = ['image_array', 'float', 'float', 'float', 'str']
        # multiple tubs
        tub_hand = TubHandler(path=cfg.DATA_PATH)
        tub = tub_hand.new_tub_writer(inputs=inputs, types=types)
        donkey_car.add(tub, inputs=inputs, outputs=["tub/num_records"],
                       run_condition='user/recording')

        # add a tub wiper that is triggered by channel 3 on the RC
        tub_wipe = TubWiper(tub, num_records=20)
        donkey_car.add(tub_wipe, inputs=['user/wiper_on'])

    # run the vehicle
    donkey_car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg):
    """
    Construct an auxiliary robotic vehicle from only the RC controllers and
    prints their values. The RC remote usually has a tuning pot for the throttle
    and steering channel. In this loop we run the controllers and simply print
    their values in order to allow centering the RC pwm signals. If there is a
    third channel on the remote we can use it for wiping bad data while
    recording, so we print its values here, too.
    """
    donkey_car = dk.vehicle.Vehicle()

    clock = Timestamp()
    donkey_car.add(clock, outputs=['timestamp'])

    # create the RC receiver
    rc_steering = RCReceiver(cfg.STEERING_RC_GPIO, invert=True)
    rc_throttle = RCReceiver(cfg.THROTTLE_RC_GPIO)
    rc_wiper = RCReceiver(cfg.DATA_WIPER_RC_GPIO, jitter=0.05, no_action=0)
    donkey_car.add(rc_steering, outputs=['user/angle', 'user/steering_on'])
    donkey_car.add(rc_throttle, outputs=['user/throttle', 'user/throttle_on'])
    donkey_car.add(rc_wiper, outputs=['user/wiper', 'user/wiper_on'])

    # create the lambda function for plotting into the shell
    def plotter(angle, steering_on, throttle, throttle_on, wiper, wiper_on):
        print('angle=%+5.4f, steering_on=%1d, throttle=%+5.4f, throttle_on=%1d '
              'wiper=%+5.4f, wiper_on=%1d' %
              (angle, steering_on, throttle, throttle_on, wiper, wiper_on))

    plotter_part = Lambda(plotter)
    # add plotter part
    donkey_car.add(plotter_part, inputs=['user/angle', 'user/steering_on',
                                         'user/throttle', 'user/throttle_on',
                                         'user/wiper', 'user/wiper_on'])

    # run the vehicle
    donkey_car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    config = dk.load_config()
    if args['drive']:
        drive(config, use_pid=args['--pid'], no_cam=args['--no_cam'])
    elif args['calibrate']:
        calibrate(config)




