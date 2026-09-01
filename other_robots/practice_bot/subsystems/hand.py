import math

import ntcore
import wpilib
from wpimath.filter import MedianFilter
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
from commands2 import Subsystem
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing

import constants
from constants import HandConstants as hc
from helpers.utilities import _get_motor_state, compare_motors


class Hand(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Hand')
        self.counter = hc.k_counter_offset  # note this should be an offset in constants
        self.default_rpm = 1000

        motor_type = rev.SparkMax.MotorType.kBrushless
        self.hand_motor = rev.SparkMax(hc.k_CANID_hand_left_leader, motor_type)
        self.hand_motor_follower = rev.SparkMax(hc.k_CANID_hand_right_follower, motor_type)

        self.motors = [self.hand_motor, self.hand_motor_follower]

        self.hand_controller = self.hand_motor.getClosedLoopController()
        self.hand_encoder = self.hand_motor.getEncoder()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = rev.ResetMode.kResetSafeParameters
        self.rev_persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash else rev.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors
        self.configs = hc.k_hand_configs

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # initialize states
        self.hand_on = False
        self.current_rpm = 0


        # self.arm_profile.reset(self.setpoint)
        # self.arm_profile.setGoal(self.setpoint)

        # the functions below this may need to use networktables
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.hand_prefix = constants.hand_prefix
        self.hand_on_pub = self.inst.getBooleanTopic(f"{self.hand_prefix}/hand_on").publish()
        self.hand_rpm_pub = self.inst.getDoubleTopic(f"{self.hand_prefix}/hand_rpm").publish()
        
        self.hand_on_pub.set(self.hand_on)
        self.hand_rpm_pub.set(self.current_rpm)

    def update_nt(self):
        self.hand_on_pub.set(self.hand_on)
        self.hand_rpm_pub.set(self.current_rpm)

    def stop_hand(self):
        self.hand_motor.set(0)

        self.hand_on = False
        self.current_rpm = 0
        self.update_nt()

    def set_hand_rpm(self, rpm=3500):
        feed_forward = min(12, 12 * rpm / 5600)
        self.hand_controller.setReference(setpoint=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        self.hand_on = True
        self.current_rpm = rpm

        self.update_nt()

    def get_rpm(self):
        return self.current_rpm

    def set_brake_mode(self, brake_on=True):
        
        idle_mode = rev.SparkBaseConfig.IdleMode.kBrake if brake_on else rev.SparkBaseConfig.IdleMode.kCoast

        # Non-persistent - just change  things temporarily - these settings leave the current config untouched
        no_resets = rev.ResetMode.kNoResetSafeParameters
        no_persists = rev.PersistMode.kNoPersistParameters

        # make a temporary config just to set break or coast
        tmp_config = rev.SparkBaseConfig().setIdleMode(idle_mode)
        print(f'Temp config on intake: {tmp_config.Presets}')  # just wondering what is in there; delete after testing

        state_before = _get_motor_state(self.deploy_motor)
        rev_errors = self.deploy_motor.configure(tmp_config, no_resets, no_persists)
        state_after = _get_motor_state(self.deploy_motor)
        # If you are paranoid you can see each state - only the idle mode changes
        # compare_motors(state_before, state_after, name_a='INTAKE BEFORE', name_b='INTAKE AFTER')

        # report our results - but not the best way since there is no timestamp here
        print(f'Setting intake to idle mode {idle_mode}: {rev_errors} at {wpilib.Timer.getFPGATimestamp():.1f}s')


    def periodic(self) -> None:
        self.counter += 1

        if self.counter % 20:
            self.update_nt()