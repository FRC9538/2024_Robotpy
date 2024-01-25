import wpilib
import wpilib.drive
from rev import CANSparkMax
# for some damn reason rev 2024.2.0 crashes on CANSparkMax init
# rev is 2024.0.0b1.post1 rn
from wpimath.geometry import Rotation2d

class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
        self.l_drive_lead   = CANSparkMax(1, CANSparkMax.MotorType.kBrushed)
        self.l_drive_follow = CANSparkMax(3, CANSparkMax.MotorType.kBrushed)
        self.r_drive_lead   = CANSparkMax(2, CANSparkMax.MotorType.kBrushed)
        self.r_drive_follow = CANSparkMax(4, CANSparkMax.MotorType.kBrushed)

        self.l_drive = wpilib.MotorControllerGroup(self.l_drive_lead,self.l_drive_follow)
        self.r_drive = wpilib.MotorControllerGroup(self.r_drive_lead, self.r_drive_follow)
        
        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_drive_lead, self.r_drive_lead)

        # For carninal plane driving, can not get it work. SparkMax error
        # self.robot_drive = wpilib.drive.MecanumDrive(self.l_drive_lead, self.l_drive_follow, self.r_drive_lead,self.r_drive_follow)
        
        #Contoller set up. We are using the OceanGate controller on port 0
        self.controller = wpilib.XboxController(0)
        
        # FOR THE LOVE OF GOD DO NOT LET ANY BE SET TO TRUE (false by default so unset is fine), invert motors HERE
        self.l_drive_lead.restoreFactoryDefaults(False)
        self.l_drive_follow.restoreFactoryDefaults(False)
        self.r_drive_lead.restoreFactoryDefaults(False)
        self.r_drive_follow.restoreFactoryDefaults(False)
        
        # self.l_drive_follow.follow(self.l_drive_lead)
        # self.r_drive_follow.follow(self.r_drive_lead)
        
        

    def teleopPeriodic(self):

        #Wokring drive
        self.robot_drive.arcadeDrive(-self.controller.getRightX() * 0.7, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * 0.7)

        # # Bassically arcade drive with an additional button that will turn Bueford
        # self.robot_drive.curvatureDrive(-self.controller.getRightX(), -self.controller.getRightY(), self.controller.getAButton())


        # # For carndinal driving, does not function with DifferentialDriving. 
        # # We need to use MecanumDriving, this has problems with SparkMax
        # # Drive using the X, Y, and Z axes of the joystick.
        # self.robot_drive.driveCartesian(-self.controller.getRightY(), -self.controller.getRightX(), -self.controller.getZ())
        # # Drive at 45 degrees relative to the robot, at the speed given by the Y axis of the joystick, with no rotation.
        # self.robot_drive.drivePolar(-self.controller.getRightY(), Rotation2d.fromDegrees(45), 0)