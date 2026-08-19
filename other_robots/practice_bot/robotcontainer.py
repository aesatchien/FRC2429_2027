# 2429 FRC code for 2026 offseason

import wpilib
import ntcore
from commands2 import InstantCommand
from wpimath.geometry import Pose2d
import commands2
from commands2.printcommand import PrintCommand

# pathplanner stuff
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.auto import NamedCommands

# 2429 helper files
import constants
from helpers import joysticks as js
from constants import AutoConstants as ac

# 2429 subsystems
from subsystems.led import Led
from subsystems.quest import Questnav
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.vision import Vision
from subsystems.targeting import Targeting

# 2429 "auto" commands - just an organizational division of commands
from autonomous.pathing_drawing import DrawingAuto


# 2429 commands
from commands.can_status import CANStatus
from commands.drive_by_velocity_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_subsystem_targeting import DriveByJoystickSubsystemTargeting
from commands.reset_field_centric import ResetFieldCentric
from commands.set_leds import SetLEDs
from commands.swerve_test import SwerveTest
from commands.swerve_set_x import SwerveSetX


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
    little robot logic should actually be handled in the :class:`.Robot` periodic methods (other than the scheduler
    calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # ----------  SUBSYSTEMS  ---------------
        # The robot's subsystems
        self.questnav = Questnav()  # going to break the silo convention and let the Swerve see the quest for now
        self.swerve = Swerve(questnav=self.questnav)
        self.targeting = Targeting(swerve=self.swerve)
        self.vision = Vision()
        # self.climber = Climber()
        self.robot_state = RobotState()  # currently has a callback that LED can register
        self.led = Led(robot_state=self.robot_state)  # may want LED last because it may want to know about other systems

        # ----------  CONTROLLERS & DEFAULTS  ---------------
        self.bind_driver_buttons()
        self.bind_codriver_buttons()  # if we need to
        self.bind_bbox_buttons()

        self.swerve.setDefaultCommand(DriveByJoystickSubsystemTargeting(
              container=self,
              swerve=self.swerve,
              driver_controller=js.play_station_controller,
              targeting=self.targeting,
              button_box=js.bbox_1,
         ))

        if not constants.k_swerve_only:
            pass

        # ----------  DASHBOARD & PATHPLANNER  ---------------
        self.register_commands()

        self.initialize_dashboard()

        self.position_index = 0

        Pathfinding.setPathfinder(LocalADStar())

    def bind_driver_buttons(self):
         # ----------  DRIVER BUTTONS  ---------------
            print("Binding driver buttons")
    
            # --- Drive & Navigation ---
    
            # Allow the driver to reset the field in case of emergency
            js.ps_triangle.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0).ignoringDisable(True))
            js.ps_triangle.debounce(0.5).onTrue(InstantCommand(lambda: self.questnav.quest_sync_odometry()).ignoringDisable(True))
    
            # start / stop tracking
            js.ps_r1.onTrue(InstantCommand(lambda: self.targeting.start_tracking())
                                .andThen(InstantCommand(lambda: self.shooter.set_shooter_rpm(rpm=sc.k_fire_up_speed))))
            js.ps_r1.onFalse(InstantCommand(lambda: self.targeting.stop_tracking()))
    
            # D-Pad: Slow, smooth robot-centric alignment (Nudge)
            dpad_driving = True
            if dpad_driving:
                dpad_output = 0.15
                js.ps_up.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(dpad_output, 0, 0), timeout=10))
                js.ps_down.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(-dpad_output, 0, 0), timeout=10))
                js.ps_left.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, dpad_output, 0), timeout=10))
                js.ps_right.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, -dpad_output, 0), timeout=10))
            else:
                pass
    
            js.ps_l2.whileTrue(SwerveSetX(container=self, swerve=self.swerve))

    def bind_codriver_buttons(self) -> None:
        # ----------  CO-DRIVER BUTTONS  ---------------
        print("Binding codriver buttons")

    def bind_bbox_buttons(self) -> None:
        print("Binding bbox buttons")

        js.bbox_1_3.debounce(.2).whileTrue(SwerveTest(container=self, swerve=self.swerve))

        # allow us to react to brownouts by lowering the current limit on the drive motors
        js.bbox_1_4.onTrue(
            commands2.ConditionalCommand(
                # brownout is ON → turn it OFF, flash green (back to full speed, 60A)
                InstantCommand(lambda: self.swerve.set_brownout_mode(False))
                    .andThen(SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS, indicator_timeout=2)),
                # brownout is OFF → turn it ON, flash red (slow down to 40A max)
                InstantCommand(lambda: self.swerve.set_brownout_mode(True))
                    .andThen(SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kFAILURE, indicator_timeout=2)),
                self.swerve.get_brownout_mode
            ).ignoringDisable(True)
        )

        # user should never sync the odometry.  should only be done with a good apriltag, not by the operator
        #js.bbox_1_4.onTrue(InstantCommand(lambda: self.questnav.quest_sync_odometry()).ignoringDisable(True))
        js.bbox_1_5.onTrue(InstantCommand(lambda: self.questnav.quest_enabled_toggle(force='off')).ignoringDisable(True))
        js.bbox_1_6.onTrue(InstantCommand(lambda: self.questnav.quest_enabled_toggle(force='on')).ignoringDisable(True))
        js.bbox_1_7.onTrue(InstantCommand(lambda: self.questnav.quest_unsync_odometry()).ignoringDisable(True))



    def initialize_dashboard(self):
        # ----------  DASHBOARD COMMANDS  ---------------
        # --------------   COMMANDS FOR GUI (ROBOT DEBUGGING) - 20250224 CJH
        command_prefix = constants.command_prefix
        # --------------   TESTING LEDS ----------------
        self.led_mode_chooser = wpilib.SendableChooser()
        [self.led_mode_chooser.addOption(key, value) for key, value in self.led.modes_dict.items()]  # add all the indicators
        self.led_mode_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(SetLEDs(container=self, led=self.led, mode=selected_value)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/LED Mode', self.led_mode_chooser)

        self.led_indicator_chooser = wpilib.SendableChooser()
        [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
            SetLEDs(container=self, led=self.led, indicator=selected_value)))
        wpilib.SmartDashboard.putData(f'{command_prefix}/LED Indicator', self.led_indicator_chooser)

        # set all subsystems - used on dash
        wpilib.SmartDashboard.putData(f'{command_prefix}/SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))


        # commands for pyqt dashboard - please do not remove
        COMMAND_LIST = [CANStatus(container=self),
                        ResetFieldCentric(container=self, swerve=self.swerve, angle=0)]
        for cmd in COMMAND_LIST:
            wpilib.SmartDashboard.putData(f'{command_prefix}/{cmd.getName()}', cmd)
        #wpilib.SmartDashboard.putData(f'{command_prefix}/CANStatus', CANStatus(container=self))

        # You can and should use the exact same list of commands in the gui to watch for
        # These are left in to demonstrate a complete UI - the real one will be full of Commands (python classes), not strings
        FAKE_COMMAND_LIST = [
            # 'IntakeReverse', 'MoveClimberDown', 'MoveClimberUp', 'ResetFlex', 'GyroReset',
            'FakeCommand']
        for cmd in FAKE_COMMAND_LIST:
            # The `lambda cmd=cmd:` is a common Python technique called the "default argument hack".
            # Lambdas in loops have "late binding," meaning they capture the variable `cmd`, not its value.
            # Without `cmd=cmd`, EVERY button would print the LAST value of `cmd` ('GyroReset')
            # because that's what `cmd` is when the loop finishes.
            # By setting `cmd=cmd` as a default argument, we force the lambda to capture
            # the *current* value of `cmd` during each iteration of the loop.
            wpilib.SmartDashboard.putData(f'{command_prefix}/{cmd}', InstantCommand(lambda cmd=cmd: print(f'Called {cmd} at {wpilib.Timer.getFPGATimestamp():.1f}s'))
                                          .alongWith(commands2.WaitCommand(2)).ignoringDisable(True))

        # end pyqt dashboard section

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.states_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            # `setattr` is the programmatic way to set an attribute. It's equivalent to
            # `self.robot_state.target = selected_value`, but can be used inside a lambda.
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: setattr(self.robot_state, 'target', selected_value))))
        wpilib.SmartDashboard.putData(f'{command_prefix}/RobotScoringMode', self.score_test_chooser)

        # ----------  AUTONOMOUS CHOOSER SECTION  ---------------
        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory - must exist 
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.auto_delay_entry = self.inst.getDoubleTopic(f"{constants.auto_prefix}/auto_delay").getEntry(0.0)
        # Publish a default so it shows up on the dashboard immediately
        self.auto_pub = self.inst.getDoubleTopic(f"{constants.auto_prefix}/auto_delay").publish()
        self.auto_pub.set(0)  # set an initial value so it shows up on the dashboard
        self.auto_chooser = wpilib.SendableChooser()  #  use this if you don't have any pathplanner autos defined
        self.auto_chooser.addOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(ac.k_auto_duration)))
        
        self.auto_chooser.setDefaultOption('2a: Drawing Auto *CODE*', DrawingAuto(self, indent=0))


        wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)  #

    def register_commands(self):
        # ----------  PATHPLANNER COMMANDS  ---------------
        # this is for PathPlanner, so it can call our commands.  Note they do not magically show up in pathplanner
        # you have to add them there, and then it remembers your list of commands.  so name them wisely
        
        NamedCommands.registerCommand('hello', commands2.PrintCommand("hello!"))
        
        
    def get_autonomous_command(self):
        cmd = self.auto_chooser.getSelected()
        delay = self.auto_delay_entry.get()
        if delay > 0.0:
            # Schedule the command independently to avoid composition ownership crashes
            # when running Auto multiple times!
            print(f"** Started {cmd.getName()} with delay of {delay} **")
            return commands2.WaitCommand(delay).andThen(
                InstantCommand(lambda: cmd.schedule())
            )
        print(f"** Started {cmd.getName()} with no delay**")
        return cmd
