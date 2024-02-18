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
        
        # invert follow motors here if needed
        self.l_drive_follow.follow(self.l_drive_lead)
        self.r_drive_follow.follow(self.r_drive_lead)
        
        # arm setup
        self.l_arm = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)
        self.r_arm = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)
        self.r_arm.setInverted(True)
        self.arm_angle = 0 # 0-90
        
        # use preferences to tune these (you can edit preferences in SmartDashboard and Shuffleboard)
        # might change this
        self.arm_controller = PIDController(self.arm_Kp, self.arm_Ki, self.arm_Kd)
        self.arm_controller.setIZone(self.arm_KiZone)
        self.arm_encoder = self.l_arm.getEncoder()
        self.arm_angle = 0
        
        # shooter setup
        self.l_shooter = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.l_shooter.setInverted(True)
        self.r_shooter = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.r_shooter.follow(self.l_shooter)
        self.shooter_timer = wpilib.Timer()
        self.shooter_speed = 1
        self.shooting = False

        self.beam_break = wpilib.DigitalInput(9)
        
        # intake setup
        self.intake = CANSparkMax(10, CANSparkMax.MotorType.kBrushless) # maybe rename
        self.intake.setInverted(True)
        self.intake_backwards_time = 0.1
        self.intake_speed = 0.5
        self.intake_running = False
        self.intake_backwards = False
        self.intake_timer = wpilib.Timer()
        self.loaded = False
        
        #Contoller setup. We are using the OceanGate controller (logitech f310) on port 0
        self.controller = wpilib.XboxController(0)
        self.rumble_timer = wpilib.Timer()
        self.rumble_time = float('inf')
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
        if not self.beam_break.get() and not self.loaded and not self.intake_backwards:
            self.intake_timer.restart()
            self.intake_running = False
            self.intake_backwards = True
            
        if self.intake_running:
            self.intake.set(self.intake_speed)
        elif not self.intake_backwards:
            self.intake.set(0)
        elif self.intake_timer.hasElapsed(self.intake_backwards_time):
            self.loaded = True
            self.intake_backwards = False
        else: 
            self.intake.set(-self.intake_speed)
         
        # SHOOTER
        if self.shooting:
            self.l_shooter.set(self.shooter_speed)
            if self.shooter_timer.hasElapsed(4):
                self.shooting = False
                self.loaded = False
            elif self.shooter_timer.hasElapsed(1): # after one second
                self.intake.set(self.intake_speed)
        else:
            self.l_shooter.set(0)
            
    
    def teleopInit(self):
        self.loadPreferences()
    
    def teleopPeriodic(self):
        # Sets speed to be left stick position when left bumper is pressed
        if self.controller.getLeftBumper():
            self.drive_speed = (-self.controller.getLeftX()+1)/2
        
        match self.drive_mode: # for fun :)
            case 0:
                # normal right stick drive
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, -self.controller.getRightY() * self.drive_speed)
            case 1:
                # mariocart shoulder trigger drive with right stick steering
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * self.drive_speed)
                
        if self.controller.getStartButtonPressed() and self.controller.getRightBumper() and not COMPETITION:
            self.drive_mode = 0 if self.drive_mode == 1 else self.drive_mode+1
        
        if self.controller.getAButtonPressed() and not self.loaded:
            self.intake_running = not self.intake_running
            
        if self.controller.getBButtonPressed() and self.loaded and not self.shooting:
            self.shooter_timer.restart()
            self.shooting = True
        
        # dpad thing pressed
        if self.controller.getPOV() != self.last_pov:
            match self.controller.getPOV():
                case 270:
                    self.arm_angle = max(0, min(90, self.arm_angle-15))
                    print("angle ", self.arm_angle)
                case 90:
                    self.arm_angle = max(0, min(90, self.arm_angle+15))
                    print("angle ", self.arm_angle)
                    
        self.last_pov = self.controller.getPOV()
            
        # arm pid is broken rn some of this was also for testing
        # arm_speed = self.arm_controller.calculate(self.arm_encoder.getPosition()*self.arm_encoder.getPositionConversionFactor(), angleToRotations(self.arm_angle))
        # arm_speed = self.arm_controller.calculate(0, angleToRotations(self.arm_angle))
        
        # if arm_speed != 0:
        #     print(arm_speed)
        # different gear ratio :P
        # self.l_arm.set(arm_speed * 0.75)
        # self.r_arm.set(arm_speed)
    
    
    def robotPeriodic(self):
        self.shooterPeriodic()
        
        
def angleToRotations(angle):
    return angle/1.8