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
        self.shooting1 = 20 #arm angle against the speaker
        self.shooting2 = 30 #arm angle three feet from speaker
        self.intake_angle = 1.5 #arm angle for intake
        self.amp_angle = 120 #arm angle for amp
        self.intake_backwards_time = 0.45
        self.intake_speed = 0.15

        # defaults
        self.arm_Kp = 0 #7.0
        self.arm_Ki = 0 #doesn't work w/ WPILib
        self.arm_KiZone = float('inf') #doesn't work w/ WPILib
        self.arm_Kd = 0 #0.01
        wpilib.Preferences.initDouble(ARMPKEY, self.arm_Kp)
        wpilib.Preferences.initDouble(ARMIKEY, self.arm_Ki)
        wpilib.Preferences.initDouble(ARMIZONEKEY, self.arm_KiZone)
        wpilib.Preferences.initDouble(ARMDKEY, self.arm_Kd)
        
        self.l_drive_lead   = CANSparkMax(1, CANSparkMax.MotorType.kBrushed)
        self.l_drive_follow = CANSparkMax(3, CANSparkMax.MotorType.kBrushed)
        self.r_drive_lead   = CANSparkMax(2, CANSparkMax.MotorType.kBrushed)
        self.r_drive_follow = CANSparkMax(4, CANSparkMax.MotorType.kBrushed)

        self.l_drive_lead.restoreFactoryDefaults(True)
        self.l_drive_follow.restoreFactoryDefaults(True)
        self.r_drive_lead.restoreFactoryDefaults(True)
        self.r_drive_follow.restoreFactoryDefaults(True)

        self.l_drive_lead.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.l_drive_follow.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.r_drive_lead.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.r_drive_follow.setIdleMode(CANSparkMax.IdleMode.kBrake)


        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_drive_lead, self.r_drive_lead)


        self.drive_mode = 0 #for fun
        self.drive_speed = 1 #3 options (0-1)
        self.bumper_cylce = 0 #changes drive speed
        self.inverse = 1 #inverses forward direction (1 or -1)

        # invert follow motors here if needed
        self.l_drive_follow.follow(self.l_drive_lead)
        self.r_drive_follow.follow(self.r_drive_lead)
        
        # arm setup
        self.l_arm = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)#ID 6: Left arm
        self.r_arm = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)#ID 7: Right arm
        self.r_arm.follow(self.l_arm, True)
        self.r_arm.burnFlash()
        self.l_arm.burnFlash()
        self.l_arm.clearFaults()
        self.r_arm.clearFaults()
        self.l_arm.restoreFactoryDefaults(True)
        self.r_arm.restoreFactoryDefaults(True)

        self.l_arm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.r_arm.setIdleMode(CANSparkMax.IdleMode.kBrake)

        
        self.l_arm.setInverted(True)
        self.r_arm.setInverted(True)

        self.arm_angle = 10 # 0-90
        self.arm_speed = 0
        
        # use preferences to tune these (you can edit preferences in SmartDashboard and Shuffleboard)
        # might change this
        self.arm_controller = PIDController(self.arm_Kp, self.arm_Ki, self.arm_Kd)
        self.arm_controller.setIZone(self.arm_KiZone)
        self.arm_encoder = self.r_arm.getEncoder()
        # self.arm_encoder.setPosition(0)

        
        # shooter setup
        self.l_shooter = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.l_shooter.restoreFactoryDefaults(True)
        self.l_shooter.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.l_shooter.setInverted(True)

        self.r_shooter = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.r_shooter.restoreFactoryDefaults(True)
        self.r_shooter.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.r_shooter.follow(self.l_shooter)
        self.shooter_timer = wpilib.Timer()
        self.shooter_speed = 1
        self.shooting = False

        self.beam_break = wpilib.DigitalInput(9)
        self.beam_break_timer = wpilib.Timer()
        
        # intake setup
        self.intake = CANSparkMax(10, CANSparkMax.MotorType.kBrushless)
        self.intake.setInverted(True)
        self.intake.restoreFactoryDefaults(True)
        self.intake.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.intake_speed = 0.5
        self.intake_running = False
        self.intake_backwards = False
        self.intake_timer = wpilib.Timer()
        self.loaded = False
        self.trigger_testing = 0 #for tiggered intake and shooter
        
        #Contoller setup. We are using the OceanGate controller (logitech f310) on port 0
        self.controller = wpilib.XboxController(0)#drive controller
        self.controller1 = wpilib.XboxController(1)#shooting controller1
        self.last_pov = -1
        self.last_pov1 = -1

        self.r_arm.burnFlash()
        self.l_arm.burnFlash()
        self.l_drive_lead.burnFlash()
        self.l_drive_follow.burnFlash()
        self.r_drive_lead.burnFlash()
        self.r_drive_follow.burnFlash()
        self.l_shooter.burnFlash()
        self.r_shooter.burnFlash()
        self.intake.burnFlash()

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
                    self.bumper_cylce = -1
            self.bumper_cylce += 1

        if self.controller.getBackButtonPressed(): #flips the forawrd direction
            self.inverse *= -1

        match self.drive_mode: # for fun :)
            case 0:#normal arcade drive 
                self.robot_drive.arcadeDrive(-1 * self.controller.getRightX() * self.drive_speed, # speed
                                             self.inverse * self.controller.getRightY() * self.drive_speed) # rotation
            case 1:#mariocart drive
                # mariocart shoulder trigger drive with right stick steering
                self.robot_drive.arcadeDrive(-self.controller.getRightX() * self.drive_speed, (self.controller.getRightTriggerAxis()-self.controller.getLeftTriggerAxis()) * self.drive_speed)
        
        #for fun        
        if self.controller.getStartButtonPressed() and self.controller.getRightBumper() and not COMPETITION:
            self.drive_mode = 0 if self.drive_mode == 1 else self.drive_mode+1
            
        if self.controller1.getAButtonPressed() and not self.loaded:
            self.intake_running = not self.intake_running
            print(self.intake_running)
        if self.controller1.getBButtonPressed() and self.loaded and not self.shooting:
            self.shooter_timer.restart()
            self.shooting = True
        
        # dpad thing pressed
        pov = self.controller.getPOV()
        if pov != self.last_pov:
            match pov:
                case 180:#down
                    self.arm_angle = max(0, min(180, self.arm_angle-1))
                    print("angle ", self.arm_angle)
                case 0:#up
                    self.arm_angle = max(0, min(180, self.arm_angle+1))
                    print("angle ", self.arm_angle)
                case 90:#right
                    self.arm_angle = 70.25 * 1.82
                #270 = left  +all the diagonlas
        self.last_pov = pov

        #d-pad for drive controller
        pov1 = self.controller1.getPOV()
        if pov1 != self.last_pov1:
            match pov1:
                case 180:#down -- intake
                    self.arm_angle = self.intake_angle
                case 0:#up -- amp
                    self.arm_angle = 120
                case 90:#right -- touching speaker
                    self.arm_angle = self.shooting1
                case 270:#left -- further from speaker
                    self.arm_angle = 30
                    # +all the diagonlas
        self.last_pov1 = pov1 #update POV

        #driving controller buttons
        if self.controller.getYButton():#shooting 1
            self.arm_encoder.setPosition(0)
            self.arm_angle = 0
        if self.controller.getAButton():#shooting 2
            self.l_arm.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.r_arm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        if self.controller.getBButton():#amplifier
            self.l_arm.setIdleMode(CANSparkMax.IdleMode.kCoast)
            self.r_arm.setIdleMode(CANSparkMax.IdleMode.kCoast)


        #for testing
        if not COMPETITION:
            self.l_shooter.set(self.controller.getLeftTriggerAxis())
            self.intake.set(self.inverse* self.controller.getRightTriggerAxis())


        #intake
        if self.beam_break_timer.advanceIfElapsed(0.04):
            self.intake_running = False
            self.intake_backwards = True
            self.beam_break_timer.stop()
            self.intake_timer.restart()

        if not self.beam_break.get() and not self.loaded and not self.intake_backwards:
            self.beam_break_timer.restart()
        if self.intake_running:
            self.intake.set(-0.8)
        elif not self.intake_backwards:
            self.intake.set(0)
        elif self.intake_timer.hasElapsed(self.intake_backwards_time):
            self.intake_timer.stop()
            self.loaded = True
            self.intake_backwards = False
        elif self.intake_backwards: # intake backwards
            self.intake.set(self.intake_speed)
            self.arm_angle = self.shooting1

        # SHOOTER
        if self.shooting:
            self.l_shooter.set(-self.shooter_speed)
            if self.shooter_timer.hasElapsed(1):
                self.shooting = False
                self.loaded = False
            elif self.shooter_timer.hasElapsed(0.7): # after one second
                self.intake.set(-1)
        else:
            self.l_shooter.set(0)


        #PID control (pls switch to rev)
        self.arm_speed = self.arm_controller.calculate(self.arm_encoder.getPosition()*self.arm_encoder.getPositionConversionFactor(), angleToRotations(self.arm_angle))
        self.error = ((self.arm_encoder.getPosition()*self.arm_encoder.getPositionConversionFactor()) - (angleToRotations(self.arm_angle))) #error

        #make into precentage
        self.arm_speed =  max(-0.45, min(0.46, self.arm_speed/100)) #caps the speed
        self.l_arm.set(self.arm_speed) #sets speed


def angleToRotations(angle):
    return angle/1.82 #the conversion ratio found with gear ratio