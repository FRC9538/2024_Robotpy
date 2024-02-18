from math import fabs
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
ARMIZONEKEY = "ArmIZone"
ARMDKEY = "ArmD"


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Preferences init
        # defaults
        self.arm_Kp = 0
        self.arm_Ki = 0
        self.arm_KiZone = float('inf')
        self.arm_Kd = 0
        wpilib.Preferences.initDouble(ARMPKEY, self.arm_Kp)
        wpilib.Preferences.initDouble(ARMIKEY, self.arm_Ki)
        wpilib.Preferences.initDouble(ARMIZONEKEY, self.arm_KiZone)
        wpilib.Preferences.initDouble(ARMDKEY, self.arm_Kd)
        
        self.l_drive_lead   = CANSparkMax(1, CANSparkMax.MotorType.kBrushed)
        self.l_drive_follow = CANSparkMax(3, CANSparkMax.MotorType.kBrushed)
        self.r_drive_lead   = CANSparkMax(2, CANSparkMax.MotorType.kBrushed)
        self.r_drive_follow = CANSparkMax(4, CANSparkMax.MotorType.kBrushed)

        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_drive_lead, self.r_drive_lead)

        self.drive_mode = 0
        self.drive_speed = 1 

        self.bumper_cylce = 0
        self.inverse = 1

        
        
        # invert follow motors here if needed
        self.l_drive_follow.follow(self.l_drive_lead)
        self.r_drive_follow.follow(self.r_drive_lead)
        
        # arm setup
        self.l_arm = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)
        self.r_arm = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)
        self.r_arm.follow(self.l_arm, True) # right arm is inverted

        self.arm_angle = 0 # 0-90
        
        # use preferences to tune these (you can edit preferences in SmartDashboard and Shuffleboard)
        # might change this
        self.arm_controller = PIDController(self.arm_Kp, self.arm_Ki, self.arm_Kd)
        self.arm_controller.setIZone(self.arm_KiZone)
        self.arm_encoder = self.l_arm.getEncoder()
        self.arm_angle = 0
        
        # shooter setup
        self.l_shooter = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.r_shooter = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.r_shooter.follow(self.l_shooter)
        self.shooter_timer = wpilib.Timer()
        self.shooter_speed = 1
        self.shooting = False

        self.beam_break = wpilib.DigitalInput(9)
        
        # intake setup
        self.intake = CANSparkMax(10, CANSparkMax.MotorType.kBrushless) # maybe rename
        self.intake_speed = 0.5
        self.intake_shoot_speed = 1
        self.intake_running = False
        self.intake_timer = wpilib.Timer()
        self.loaded = False
        
        #Contoller setup. We are using the OceanGate controller (logitech f310) on port 0
        self.controller = wpilib.XboxController(0)
        self.last_pov = -1
        


    def loadPreferences(self):
        if self.arm_Kp != wpilib.Preferences.getDouble(ARMPKEY, self.arm_Kp):
            self.arm_Kp = wpilib.Preferences.getDouble(ARMPKEY, self.arm_Kp)
            self.arm_controller.setP(self.arm_Kp)
        
        if self.arm_Ki != wpilib.Preferences.getDouble(ARMIKEY, self.arm_Ki):
            self.arm_Ki = wpilib.Preferences.getDouble(ARMIKEY, self.arm_Ki)
            self.arm_controller.setI(self.arm_Ki)
        
        if self.arm_KiZone != wpilib.Preferences.getDouble(ARMIZONEKEY, self.arm_KiZone):
            self.arm_KiZone = wpilib.Preferences.getDouble(ARMIZONEKEY, self.arm_KiZone)
            self.arm_controller.setIZone(self.arm_KiZone)
        
        if self.arm_Kd != wpilib.Preferences.getDouble(ARMDKEY, self.arm_Kd):
            self.arm_Kd = wpilib.Preferences.getDouble(ARMDKEY, self.arm_Kd)
            self.arm_controller.setD(self.arm_Kd)
    


    def shooterPeriodic(self):
        # INTAKE
        print(self.beam_break.get())
        if self.beam_break.get() == True:
            self.intake_timer.restart()
            self.loaded = True
            self.intake_running = False
        
        if self.intake_timer.get() < 0.1 and self.intake_running:
            self.intake.set(-self.intake_speed)
        elif self.intake_timer.get() >= 0.1 :
            self.loaded = True
        else: # safety
            if self.intake_running:
                # self.intake.set(self.intake_speed)
                print("wrong")
            else:
                self.intake.set(0)
                
        # SHOOTER
        if self.shooting:
            self.l_shooter.set(self.shooter_speed)
            if self.shooter_timer.get() >= 1: # after one second
                self.intake.set(self.intake_speed)
                
            if self.shooter_timer.get() >= 4:
                self.shooting = False
        else:
            self.l_shooter.set(0)
            
    
    def teleopInit(self):
        self.loadPreferences()
    
    def teleopPeriodic(self):
        
        # Cycles through different drive speeds on left bumper press
        if self.controller.getLeftBumperPressed():
            match self.bumper_cylce:
                case 0:
                    self.drive_speed = 1
                case 1:
                    self.drive_speed = 0.75
                case 2: 
                    self.drive_speed = 0.5
                case 3:
                    self.drive_speed = 0.3
                    self.bumper_cylce = -1
            self.bumper_cylce += 1
        
        match self.drive_mode: # for fun :)
            case 0:#normal arcade drive 
                self.inverse = self.inverse
                if self.controller.getRawButtonPressed: #flips the forawrd direction
                    self.inverse *= -1
                rotation_precentage = 1 - abs(self.controller.getRightX())
                self.robot_drive.arcadeDrive(a* self.controller.getLeftX() * self.drive_speed,a* self.controller.getLeftY()*self.drive_speed* rotation_precentage)
            case 1:#mariocart drive
                # mariocart shoulder trigger drive with right stick steering
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * self.drive_speed)
        #for fun        
        if self.controller.getStartButtonPressed() and self.controller.getRightBumper() and not COMPETITION:
            self.drive_mode = 0 if self.drive_mode == 1 else self.drive_mode+1
        #intake on if not already on
        # if self.controller.getAButtonPressed() and not self.loaded:
        #     self.intake_running = (not self.intake_running)
        #     self.intake_timer.restart()
        #shooter on if loaded
        # if self.controller.getBackButtonPressed() and self.loaded:
        #     self.shooter_timer.restart()
        #     self.shooting = True
        
        # self.shooterPeriodic()
        self.intake.set(-self.controller.getLeftTriggerAxis())
        self.l_shooter.set(-self.controller.getRightTriggerAxis())
        
        # dpad thing pressed
        if self.controller.getPOV() != self.last_pov:
            match self.controller.getPOV():
                case 180:
                    self.arm_angle = max(0, min(110, self.arm_angle-5))
                    print("angle ", self.arm_angle)
                case 0:
                    self.arm_angle = max(0, min(110, self.arm_angle+5))
                    print("angle ", self.arm_angle)
                case 90:
                    self.arm_angle = 70.25 * 1.82

                    
        self.last_pov = self.controller.getPOV()
        
        if self.controller.getBButton():
            self.arm_angle = 40
        if self.controller.getYButton():
            self.arm_angle = 60
            print(self.arm_encoder.getPosition())
        if self.controller.getXButton():
            self.arm_angle = 100
            print(self.arm_encoder.getPosition())
        if self.controller.getAButton():
            self.arm_angle = 10

        # arm pid is broken rn some of this was also for testing
        arm_speed = self.arm_controller.calculate(self.arm_encoder.getPosition()*self.arm_encoder.getPositionConversionFactor(), angleToRotations(self.arm_angle))
        # arm_speed = self.arm_controller.calculate(0, angleToRotations(self.arm_angle)) 
        self.error = ((self.arm_encoder.getPosition()*self.arm_encoder.getPositionConversionFactor()) - (angleToRotations(self.arm_angle)))

        #make into precentage

        self.arm_controller.setTolerance

        arm_speed =  max(-0.45, min(0.46, arm_speed/100))

        # different gear ratio :P
        self.l_arm.set(arm_speed)
def angleToRotations(angle):
    return angle/1.82