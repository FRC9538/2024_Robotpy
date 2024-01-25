from venv import logger
import wpilib
import wpilib.drive
from rev import CANSparkMax
# for some damn reason rev 2024.2.0 crashes on CANSparkMax init
# rev is 2024.0.0b1.post1 rn

COMPETITION = False

class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
        self.l_drive_lead   = CANSparkMax(1, CANSparkMax.MotorType.kBrushed)
        self.l_drive_follow = CANSparkMax(3, CANSparkMax.MotorType.kBrushed)
        self.r_drive_lead   = CANSparkMax(2, CANSparkMax.MotorType.kBrushed)
        self.r_drive_follow = CANSparkMax(4, CANSparkMax.MotorType.kBrushed)

        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_drive_lead, self.r_drive_lead)

        self.drive_mode = 0
        self.drive_speed = 1

        # For carninal plane driving, can not get it work. SparkMax error
        # self.robot_drive = wpilib.drive.MecanumDrive(self.l_drive_lead, self.l_drive_follow, self.r_drive_lead,self.r_drive_follow)
        
        #Contoller set up. We are using the OceanGate controller on port 0
        self.controller = wpilib.XboxController(0)
        
        # FOR THE LOVE OF GOD DO NOT LET ANY BE SET TO TRUE (false by default so unset is fine), invert motors HERE
        self.l_drive_lead.restoreFactoryDefaults(False)
        self.l_drive_follow.restoreFactoryDefaults(False)
        self.r_drive_lead.restoreFactoryDefaults(False)
        self.r_drive_follow.restoreFactoryDefaults(False)
        
        self.l_drive_follow.follow(self.l_drive_lead)
        self.r_drive_follow.follow(self.r_drive_lead)
        

    def teleopPeriodic(self):
        match self.drive_mode:
            case 0:
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, -self.controller.getRightY() * self.drive_speed)
            case 1:
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * self.drive_speed)
                
        if self.controller.getStartButtonPressed() and self.controller.getRightBumper() and not COMPETITION:
            self.drive_mode = 0 if self.drive_mode == 1 else self.drive_mode+1
        
        
        # Basically arcade drive with an additional button that will turn Bueford