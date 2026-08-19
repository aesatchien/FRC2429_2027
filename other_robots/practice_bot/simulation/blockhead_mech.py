import math
import wpilib
from wpilib import Color, Color8Bit
from wpimath.units import inchesToMeters
import constants

class BlockheadMech:
    """
    A class to manage the Mechanism2d visualizations for the robot.
    This replaces the script-based simmech.py and provides an API for
    updating mechanism states.
    """

    def __init__(self):
        """
        Initialize the Mechanism2d objects and publish them to SmartDashboard.
        """
        # Dimensions
        self.width = inchesToMeters(40)  # inches (approx robot length + intake)
        self.height = inchesToMeters(30) # inches (approx max height)
        
        # ----------------- Mechanism2d Views -----------------
        # Side view is best for Intake, Elevator, Shooter, Climber
        self.mech_side = wpilib.Mechanism2d(self.width + inchesToMeters(10), self.height + inchesToMeters(10))

        # ----------------- Visual Constants -----------------
        self.color_chassis = Color8Bit(Color.kGray)
        self.color_wheel = Color8Bit(Color.kWhite)

        self.line_weight = 10 # 1 inch approx 10 pixels?
        # self.color_climber = Color8Bit(Color.kGreen)
        
        # ----------------- Initialization -----------------
        self._init_structure()

        # self._init_climber()

        # Put to dashboard
        # mech_prefix = constants.mech_prefix
        wpilib.SmartDashboard.putData("Mech Side View", self.mech_side)

    def _get_rel_angle(self, target_abs, parent_abs):
        """
        Calculates the relative angle needed for a ligament given the 
        desired absolute angle and the parent's absolute angle.
        """
        return target_abs - parent_abs

    def _init_structure(self):
        # Drivetrain
        """Stub: Base chassis line."""
        # Robot starting X offset
        self.start_x = inchesToMeters(5)
        
        # Chassis: 2" bar, 27" long. Top at 4.5".
        # Center of bar is at 3.5" (since it's 2" thick, 2.5" to 4.5")
        self.root_chassis = self.mech_side.getRoot("chassis_root", self.start_x, inchesToMeters(3.5))
        self.chassis_ligament = self.root_chassis.appendLigament(
            "chassis", inchesToMeters(27), 0, 20, self.color_chassis
        )

        # Wheels: 4" diameter (2" radius). Center at Y=2.
        # 5" in from edges (0 and 27). So at 5" and 22".
        
        # Rear Wheel
        self.root_rear_wheel = self.mech_side.getRoot("rear_wheel", self.start_x + inchesToMeters(5), inchesToMeters(2))
        self.rear_wheel_ligament = self.root_rear_wheel.appendLigament(
            "rear_spoke", inchesToMeters(2), 0, 6, self.color_wheel
        )
        self.rear_wheel_ligament_2 = self.root_rear_wheel.appendLigament(
            "rear_spoke_2", inchesToMeters(2), 180, 6, self.color_wheel
        )
        
        # Front Wheel
        self.root_front_wheel = self.mech_side.getRoot("front_wheel", self.start_x + inchesToMeters(22), inchesToMeters(2))
        self.front_wheel_ligament = self.root_front_wheel.appendLigament(
            "front_spoke", inchesToMeters(2), 0, 6, self.color_wheel
        )
        self.front_wheel_ligament_2 = self.root_front_wheel.appendLigament(
            "front_spoke_2", inchesToMeters(2), 180, 6, self.color_wheel
        )


    '''def _init_climber(self):
        """Stub: Extension arms."""
        # Attached to chassis center/back
        self.climber_root_y = inchesToMeters(2)
        self.root_climber = self.mech_side.getRoot("climber_root", self.start_x + inchesToMeters(2), self.climber_root_y)
        self.abs_climber = 90
        self.climber_stage_1 = self.root_climber.appendLigament(
            "climber_stage_1", inchesToMeters(15), self._get_rel_angle(self.abs_climber, 0), self.line_weight, self.color_climber
        )
        
        # Hook pointing left (180 abs)
        self.abs_hook = 180
        self.climber_hook = self.climber_stage_1.appendLigament(
            "climber_hook", inchesToMeters(4), self._get_rel_angle(self.abs_hook, self.abs_climber), self.line_weight, self.color_wheel
        )
    '''

    # ---------------- Update Methods ----------------

    def update_drivetrain(self, speed_mps):
        # Rotate wheels based on speed. 
        # Speed (m/s) -> angular velocity. 
        # Just adding a factor to animate it.
        rotation_step = speed_mps * 10 # arbitrary scaling
        
        new_rear_angle = self.rear_wheel_ligament.getAngle() + rotation_step
        self.rear_wheel_ligament.setAngle(new_rear_angle)
        self.rear_wheel_ligament_2.setAngle(new_rear_angle + 180)
        
        new_front_angle = self.front_wheel_ligament.getAngle() + rotation_step
        self.front_wheel_ligament.setAngle(new_front_angle)
        self.front_wheel_ligament_2.setAngle(new_front_angle + 180)

    

    '''def update_climber(self, height_from_ground: float):
        # Calculate length: Target Height - Root Height
        length = max(0.0, height_from_ground - self.climber_root_y)
        self.climber_stage_1.setLength(length)
    '''
