import wpilib
import wpilib.drive
from wpimath.controller import PIDController
from rev import CANSparkMax
# for some damn reason rev 2024.2.0 crashes on CANSparkMax init
# rev is 2024.0.0b1.post1 rn

COMPETITION = False

# preferences
ARMPKEY = "ArmP"
ARMIKEY = "ArmI"
ARMDKEY = "ArmD"


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Preferences init
        # defaults
        self.arm_Kp = 0
        self.arm_Ki = 0
        self.arm_Kd = 0
        wpilib.Preferences.initDouble(ARMPKEY, self.arm_Kp)
        wpilib.Preferences.initDouble(ARMIKEY, self.arm_Ki)
        wpilib.Preferences.initDouble(ARMDKEY, self.arm_Kd)
        
        self.l_drive_lead   = CANSparkMax(1, CANSparkMax.MotorType.kBrushed)
        self.l_drive_follow = CANSparkMax(3, CANSparkMax.MotorType.kBrushed)
        self.r_drive_lead   = CANSparkMax(2, CANSparkMax.MotorType.kBrushed)
        self.r_drive_follow = CANSparkMax(4, CANSparkMax.MotorType.kBrushed)

        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_drive_lead, self.r_drive_lead)

        self.drive_mode = 0
        self.drive_speed = 1 

        # For carninal plane driving, can not get it work. SparkMax error
        # self.robot_drive = wpilib.drive.MecanumDrive(self.l_drive_lead, self.l_drive_follow, self.r_drive_lead,self.r_drive_follow)
        
        # FOR THE LOVE OF GOD DO NOT LET ANY BE SET TO TRUE (false by default so unset is fine)
        # This is not needed it was for testing so please remove if you plan on configuring the spark maxs at all or just if you feel like it
        self.l_drive_lead.restoreFactoryDefaults(False)
        self.l_drive_follow.restoreFactoryDefaults(False)
        self.r_drive_lead.restoreFactoryDefaults(False)
        self.r_drive_follow.restoreFactoryDefaults(False)
        
        # invert follow motors here if needed
        self.l_drive_follow.follow(self.l_drive_lead)
        self.r_drive_follow.follow(self.r_drive_lead)
        
        # arm setup
        self.l_arm = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)
        self.r_arm = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)
        self.r_arm.follow(self.l_arm)
        
        # use preferences to tune these (you can edit preferences in SmartDashboard and Shuffleboard)
        # might change this
        self.arm_pid = PIDController(self.arm_Kp, self.arm_Ki, self.arm_Kd)
        # probablly enable continuous input but i dont know the angles of the arm and stuff yet
        
        # shooter setup
        self.l_shooter = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.r_shooter = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.r_shooter.follow(self.l_shooter)
        
        self.shooter_speed = 0 # 0-1

        # intake setup
        self.intake = CANSparkMax(10, CANSparkMax.MotorType.kBrushless) # maybe rename
        
        self.intake_speed = 0 # 0-1

        
        #Contoller setup. We are using the OceanGate controller (logitech f310) on port 0
        self.controller = wpilib.XboxController(0)
        
        # what the left stick controls
        # 0 - nothing
        # 1 - shooter speed
        # 2 - intake speed
        # 3 - arm angle
        # 4 - drive speed
        # when changed there is no way to go back to 0 currently
        self.lstick_operation = 0 
    
    def loadPreferences(self):
        if self.arm_Kp != wpilib.Preferences.getDouble(ARMPKEY, self.arm_Kp):
            self.arm_Kp = wpilib.Preferences.getDouble(ARMPKEY, self.arm_Kp)
            self.arm_pid.setP(self.arm_Kp)
        
        if self.arm_Ki != wpilib.Preferences.getDouble(ARMIKEY, self.arm_Ki):
            self.arm_Ki = wpilib.Preferences.getDouble(ARMIKEY, self.arm_Ki)
            self.arm_pid.setI(self.arm_Ki)
        
        if self.arm_Kd != wpilib.Preferences.getDouble(ARMDKEY, self.arm_Kd):
            self.arm_Kd = wpilib.Preferences.getDouble(ARMDKEY, self.arm_Kd)
            self.arm_pid.setD(self.arm_Kd)
    
    def teleopInit(self):
        self.loadPreferences()
    
    def teleopPeriodic(self):
        match self.drive_mode: # for fun :)
            case 0:
                # normal right stick drive
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, -self.controller.getRightY() * self.drive_speed)
            case 1:
                # mariocart shoulder trigger drive with right stick steering
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * self.drive_speed)
                
        if self.controller.getStartButtonPressed() and self.controller.getRightBumper() and not COMPETITION:
            self.drive_mode = 0 if self.drive_mode == 1 else self.drive_mode+1
            
        match self.lstick_operation:
            case 1: # shooter speed
                pass
            case 2: # intake speed
                pass
            case 3: # arm angle
                pass
            case 4: # drive speed
                # Sets speed to be left stick position when left bumper is pressed
                if self.controller.getLeftBumper():
                    self.drive_speed = ((-self.controller.getLeftX())+1)/2
        
        self.l_shooter.set(self.shooter_speed)
        self.intake.set(self.intake_speed)
            
        # dpad thing pressed
        if self.controller.getPOV() != -1:
            pass
            # set lstick_operation here
            # only in each sides 45 degree angle maybe more
            # (ignore inbetween)
        
        # THIS IS FOR TESTING DELETE WHEN DPAD THING WORKS or not if you like it
        if self.controller.getAButtonPressed():
            self.lstick_operation = 1 if self.lstick_operation == 4 else self.lstick_operation+1
        