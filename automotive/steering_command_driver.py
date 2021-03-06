# Do not run this program directly; only use the compiled form in bazel-bin.

"""Publishes steering commands over LCM.
"""

import argparse
import collections
import copy
import math
import os
import sys
import time

try:
    import pygame
except ImportError:
    # We will flag this as an error later, and only if we really needed it.
    pass

import lcm

from drake.lcmt_driving_command_t import lcmt_driving_command_t as lcm_msg
SteeringThrottleBrake = collections.namedtuple(
    'SteeringThrottleBrake', ['steering_angle', 'throttle', 'brake'])

STEERING_AXIS = 0
ACCEL_AXIS = 1
BRAKE_AXIS = 2

MAX_STEERING_ANGLE = math.radians(45)
STEERING_BUTTON_STEP_ANGLE = MAX_STEERING_ANGLE / 100.0
TURN_LEFT_SIGN = 1.0
TURN_RIGHT_SIGN = -1.0

# Target velocity 60mph, i.e. ~26.8224m/sec.
MAX_VELOCITY = 26.8224
THROTTLE_SCALE = MAX_VELOCITY / 300.0

MAX_BRAKE = MAX_VELOCITY
BRAKE_SCALE = THROTTLE_SCALE


def _limit_steering(requested_value):
    if abs(requested_value) <= MAX_STEERING_ANGLE:
        return requested_value
    else:
        return math.copysign(MAX_STEERING_ANGLE, requested_value)


def _limit_throttle(requested_value):
    """Apply the lower and upper limits to @p requested value."""
    if 0 <= requested_value <= MAX_VELOCITY:
        return requested_value
    elif requested_value < 0:
        return 0
    else:
        return MAX_VELOCITY


def _limit_brake(requested_value):
    if 0 <= requested_value <= MAX_BRAKE:
        return requested_value
    elif requested_value < 0:
        return 0
    else:
        return MAX_BRAKE

class KeyboardEventProcessor:
    def __init__(self):
        pygame.event.set_allowed(None)
        pygame.event.set_allowed([pygame.QUIT, pygame.KEYUP, pygame.KEYDOWN])
        pygame.key.set_repeat(100, 10)
        self.throttle_gradient = 0
        self.brake_gradient = 0
        self.keep_current_throttle_brake = False

    def processEvent(self, event, last_msg):
        new_msg = copy.copy(last_msg)

        if event.type == pygame.KEYUP and not self.keep_current_throttle_brake:
            if hasattr(event, 'key'):
                if (event.key == pygame.K_SPACE):
                    self.keep_current_throttle_brake = True
                    return new_msg
                if (event.key == pygame.K_UP):
                    self.throttle_gradient = -1
                elif (event.key == pygame.K_DOWN):
                    self.brake_gradient = -1
            # Post a fake KEYUP event so the throttle/brake can keep decreasing
            # in the absence of real (and impossible) successive key releases.
            # Yield to any KEYDOWN event waiting in the queue.
            if not pygame.event.peek(pygame.KEYDOWN):
                dummyKeyUpEvent = pygame.event.Event(pygame.KEYUP)
                pygame.event.post(dummyKeyUpEvent)

        if (event.type == pygame.KEYDOWN):
            self.keep_current_throttle_brake = False
            if (event.key == pygame.K_SPACE):
                self.keep_current_throttle_brake = True
                self.throttle_gradient = 0
                self.brake_gradient = 0
            elif (event.key == pygame.K_UP):
                self.throttle_gradient = 1
            elif (event.key == pygame.K_DOWN):
                self.brake_gradient = 1
            elif (event.key == pygame.K_LEFT):
                new_msg = new_msg._replace(steering_angle=_limit_steering(
                    last_msg.steering_angle + (
                        STEERING_BUTTON_STEP_ANGLE * TURN_LEFT_SIGN)))
            elif (event.key == pygame.K_RIGHT):
                new_msg = new_msg._replace(steering_angle=_limit_steering(
                    last_msg.steering_angle + (
                        STEERING_BUTTON_STEP_ANGLE * TURN_RIGHT_SIGN)))

        new_msg = new_msg._replace(
            throttle=_limit_throttle(
                new_msg.throttle + self.throttle_gradient * THROTTLE_SCALE),
            brake=_limit_brake(
                new_msg.brake + self.brake_gradient * BRAKE_SCALE))

        return new_msg


class JoystickEventProcessor:
    def __init__(self, joy_name):
        pygame.event.set_allowed(None)
        pygame.event.set_allowed([pygame.QUIT, pygame.JOYAXISMOTION])
        if pygame.joystick.get_count() == 0:
            pygame.quit()
            sys.exit('ERROR: No joysticks detected')
        joysticks = [pygame.joystick.Joystick(x)
                     for x in xrange(pygame.joystick.get_count())]
        self.joystick = None
        for joystick in joysticks:
            if joystick.get_name() == joy_name:
                self.joystick = joystick
                break
        if self.joystick is None:
            pygame.quit()
            sys.exit('ERROR: Joystick with system name "%s" not detected' %
                     (joy_name))
        self.joystick.init()

    def processEvent(self, event, last_msg):
        new_msg = copy.copy(last_msg)
        if event.axis == STEERING_AXIS:
            new_msg.steering_angle = (
                TURN_RIGHT_SIGN * event.value * MAX_STEERING_ANGLE)
        elif event.axis == ACCEL_AXIS:
            new_msg.throttle = _limit_throttle(-0.5 * event.value + 0.5)
        elif event.axis == BRAKE_AXIS:
            new_msg.brake = _limit_brake(-0.5 * event.value + 0.5)
        return new_msg


class bcolors:
    OKBLUE = '\033[94m'
    ENDC = '\033[0m'


def make_driving_command(throttle, steering_angle):
    msg = lcm_msg()
    msg.acceleration = throttle
    msg.steering_angle = steering_angle
    return msg


class SteeringCommandPublisher:
    def __init__(self, input_method, lcm_tag, joy_name):
        print 'Initializing...'
        pygame.init()
        self.screen = pygame.display.set_mode((300, 70))
        pygame.display.set_caption(lcm_tag)
        self.font = pygame.font.SysFont('Courier', 20)
        if input_method == 'keyboard':
            self.event_processor = KeyboardEventProcessor()
            print bcolors.OKBLUE + '--- Keyboard Control Instruction --- '\
                + bcolors.ENDC
            print 'To increase the throttle/brake: press and hold the Up/Down'\
                + ' Arrow'
            print 'To decrease the throttle/brake: release the Up/Down Arrow'
            print 'To keep the the current throttle/brake: press the Space Bar'
            print 'To increase left/right steering: press the Left/Right Arrow'
            print bcolors.OKBLUE + '------------------------------------ ' \
                + bcolors.ENDC
        else:
            self.event_processor = JoystickEventProcessor(joy_name)
        self.last_value = SteeringThrottleBrake(0, 0, 0)
        self.lc = lcm.LCM()
        self.lcm_tag = lcm_tag
        print 'Ready'

    def printLCMValues(self):
        self.screen.fill(5)
        surface = self.font.render(
            'Steering Angle: %f' % (self.last_value.steering_angle),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 0))
        surface = self.font.render(
            'Throttle Value: %f' % (self.last_value.throttle),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 22))
        surface = self.font.render(
            'Brake Value   : %f' % (self.last_value.brake),
            True, (250, 250, 250))
        self.screen.blit(surface, (2, 44))
        pygame.display.flip()

    def start(self):
        self.printLCMValues()
        while True:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            else:
                self.last_value = self.event_processor.processEvent(
                    event, self.last_value)
                msg = make_driving_command(self.last_value.steering_angle,
                                           self.last_value.throttle -
                                           self.last_value.brake)
                self.lc.publish(self.lcm_tag, msg.encode())
                self.printLCMValues()

# Looks for a file in file_path/DRIVING_COMMAND_[car_name].txt
# and reads the commands from that file. The file must be
# composed of lines of the form "command num"
# command should be a string in [up, down, left, right, cruise]
# num must be a positive float or int, and must be monotonically
# increasing because they represent time-stamps.
# We follow commands one at a time until we pass its num second-mark
class CommandsPublisher:
    def __init__(self, lcm_tag):
        self.last_value = SteeringThrottleBrake(0, 0, 0)
        self.lc = lcm.LCM()
        self.lcm_tag = lcm_tag
        # TODO : change path with your own path to drake
        self.file_path = "/Users/justinej/Documents/drake/automotive/" + self.lcm_tag + ".txt"
        self.commands = self.read_file(self.file_path)
        self.time_interval = 0.01 # seconds between steps

    def read_file(self, path_to_file):
        f = open(path_to_file, "r")
        commands = []
        previous_time_stamp = 0
        for line in f:
            # Ignore lines starting with hash-tag
            if line.startswith("#"):
                continue
            command = line[:-1].split(" ")
            command[1] = float(command[1])

            assert len(command) == 2
            assert command[0] in ['up', 'down', 'left', 'right', 'cruise']
            assert command[1] > previous_time_stamp
            previous_time_stamp = command[1]

            commands.append(command)
        f.close()
        return commands

    def speed_up(self, last_msg):
        new_msg = copy.copy(last_msg)
        throttle_gradient = 1
        new_msg = new_msg._replace(
            throttle=_limit_throttle(
                new_msg.throttle + throttle_gradient * THROTTLE_SCALE))
        return new_msg

    def slow_down(self, last_msg):
        new_msg = copy.copy(last_msg)
        brake_gradient = 1
        new_msg = new_msg._replace(
            brake=_limit_brake(
                new_msg.brake + brake_gradient * BRAKE_SCALE))
        return new_msg

    def turn_left(self, last_msg):
        new_msg = copy.copy(last_msg)
        new_msg = new_msg._replace(steering_angle=_limit_steering(
            last_msg.steering_angle + (
                STEERING_BUTTON_STEP_ANGLE * TURN_LEFT_SIGN)))
        return new_msg

    def turn_right(self, last_msg):
        new_msg = copy.copy(last_msg)
        new_msg = new_msg._replace(steering_angle=_limit_steering(
            last_msg.steering_angle + (
                STEERING_BUTTON_STEP_ANGLE * TURN_RIGHT_SIGN)))
        return new_msg

    def cruise(self):
        return self.reset()

    # Reset command sent after switching buttons
    def reset(self):
        return SteeringThrottleBrake(0, 0, 0)

    # Slow down when you've reached the end of the commands
    def end(self, last_msg):
        return self.slow_down(last_msg)

    def start(self):
        start_time = time.time()
        i = 0 # line we're reading in commands file
        while True:
            current_time = time.time() - start_time
            while (i < len(self.commands) and
                self.commands[i][1] < current_time):
                # "key is up" so we reset throttle and steering
                self.last_value = self.reset()
                i += 1

            if i == len(self.commands):
                command = ["default"]
            else: command = self.commands[i]

            if command[0] == "up":
                self.last_value = self.speed_up(self.last_value)
            elif command[0] == "down":
                self.last_value = self.slow_down(self.last_value)
            elif command[0] == "left":
                self.last_value = self.turn_left(self.last_value)
            elif command[0] == "right":
                self.last_value = self.turn_right(self.last_value)
            elif command[0] == "cruise":
                self.last_value = self.cruise()
            else:
                self.last_value = self.end(self.last_value)

            msg = lcm_msg()
            msg.acceleration = (self.last_value.throttle -
                                self.last_value.brake)
            msg.steering_angle = self.last_value.steering_angle
            self.lc.publish(self.lcm_tag, msg.encode())
            time.sleep(self.time_interval) # Repeat every self.time_interval seconds

def publish_driving_command(lcm_tag, throttle, steering_angle):
    lc = lcm.LCM()
    last_msg = make_driving_command(throttle, steering_angle)
    lc.publish(lcm_tag, last_msg.encode())


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--lcm_tag', default='DRIVING_COMMAND',
        help='tag to publish the LCM messages with')
    parser.add_argument(
        '--mode', choices=['interactive', 'one-time'], default='interactive',
        help='whether to run interactively with pygame input,'
        ' or just send a single command')
    parser.add_argument(
        '--input_method', choices=['joystick', 'keyboard', 'commands'], default='keyboard',
        help='the interactive input method to use for publishing LCM commands')
    parser.add_argument(
        '--joy_name', default='Driving Force GT',
        help='system name of the joystick')
    parser.add_argument(
        '--throttle', type=float, default='0.0', help='initial throttle')
    parser.add_argument(
        '--steering-angle', type=float, default='0.0',
        help='initial steering angle (in radians), positive-left')

    args = parser.parse_args()

    if args.mode == 'one-time':
        publish_driving_command(
            args.lcm_tag, args.throttle, args.steering_angle)
        return 0

    if 'pygame' not in sys.modules:
        print >>sys.stderr, 'error: missing pygame; see README.md for help.'
        return 1

    if args.input_method == "commands":
        publisher = CommandsPublisher(args.lcm_tag)
    else:
        publisher = SteeringCommandPublisher(
                        args.input_method, args.lcm_tag, args.joy_name)
    publisher.start()

    return 0


if __name__ == '__main__':
    sys.exit(main())
