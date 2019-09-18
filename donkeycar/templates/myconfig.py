# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

# CAMERA
IMAGE_W = 192
IMAGE_H = 144

# STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 260
STEERING_RIGHT_PWM = 480
STEERING_RC_GPIO = 26

# THROTTLE
THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM = 480
THROTTLE_STOPPED_PWM = 385
THROTTLE_REVERSE_PWM = 295
THROTTLE_RC_GPIO = 20

# DATA WIPER
DATA_WIPER_RC_GPIO = 19

# PID CONTROLLER
PID_P = 0.3
PID_I = 0.8
PID_D = 0.0

# ODOMETER
MAX_SPEED = 4.4

# TUB SETTINGS
RECORD_DURING_AI = True

# TRAINING
EARLY_STOP_PATIENCE = 10
MAX_EPOCHS = 200
USE_SPEED_FOR_MODEL = True
BATCH_SIZE = 32
