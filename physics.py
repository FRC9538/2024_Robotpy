
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import rev
from wpilib import Field2d
import wpilib.simulation
import wpimath.geometry

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.field = Field2d()
        self.physics_controller = physics_controller

        # Motors
        
        self.l_motor = robot.l_drive_lead
        self.r_motor = robot.r_drive_lead
        
        bumper_width = 3.25 * units.inch

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,             # motor configuration
            75 * units.lbs,                       # robot mass
            8.45,                                 # drivetrain gear ratio \omega In/\omega Out
            6,                                    # motors per side
            23.25 * units.inch,                   # robot wheelbase
            27 * units.inch + bumper_width * 2,   # robot width
            32.5 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                       # wheel diameter
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        
        # dont like this too much :/
        transform = self.drivetrain.calculate(self.l_motor.get(), self.r_motor.get(), tm_diff)
        self.field.setRobotPose(self.physics_controller.move_robot(transform)) 

