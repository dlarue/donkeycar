import pytest
from pytest import approx

from .setup import on_pi
from donkeycar.parts.actuator import PCA9685, PWMSteering, map_frange


@pytest.mark.skipif(on_pi() is False, reason='Not on RPi')
def test_PCA9685():
    c = PCA9685(0)


@pytest.mark.skipif(on_pi() is False, reason='Not on RPi')
def test_PWMSteering():
    c = PCA9685(0)
    s = PWMSteering(c)


def test_map_frange():
    p1 = (dict(x=4, X_min=2, X_max=8, Y_min=-9, Y_max=3), -5)
    p2 = (dict(x=3.16, X_min=3., X_max=4., Y_min=1000., Y_max=2000.), 1160.0)
    p3 = (dict(x=2.1, X_min=0.7, X_max=3.5, Y_min=-20., Y_max=20.), 0.0)
    p4 = (dict(x=14.4, X_min=14.4, X_max=17.7, Y_min=-2., Y_max=0.), -2.0)
    p5 = (dict(x=-1.1, X_min=-2.2, X_max=-1.1, Y_min=33.0, Y_max=44.0), 44.0)
    for p in (p1, p2, p3, p4, p5):
        assert map_frange(**p[0]) == approx(p[1])
