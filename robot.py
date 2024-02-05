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
        self.arm_KiZone = 0
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
        self.r_arm.setInverted(True)
        
        # use preferences to tune these (you can edit preferences in SmartDashboard and Shuffleboard)
        # might change this

        #left arm is set at 75% of right arm
        self.arm_ratio = 0.75

        # unused, testing WPI PID control
        self.arm_controller = PIDController(self.arm_Kp, self.arm_Ki, self.arm_Kd)
        self.arm_controller.setIZone(self.arm_KiZone)

        self.arm_encoder = self.l_arm.getEncoder()
        self.arm_angle = 0
        #for P
        self.arm_tick_constant = 360/200
        #for I
        self.arm_last_timestamp = wpilib.getTime()
        self.arm_error_sum = 0
        #for D
        self.arm_last_error = 0
        
        # shooter setup
        self.l_shooter = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)
        self.r_shooter = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.r_shooter.follow(self.l_shooter)

        # intake setup
        self.intake = CANSparkMax(10, CANSparkMax.MotorType.kBrushless) # maybe rename
        
        #Contoller setup. We are using the OceanGate controller (logitech f310) on port 0
        self.controller = wpilib.XboxController(0)
        
    def loadPreferences(self):
        # Uploads PID constants from smart dashbaord on driver station
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
            
        # conflicts with drive mode 2
        self.l_shooter.set(self.controller.getLeftTriggerAxis())
        self.intake.set(self.controller.getRightTriggerAxis())
        
        # dpad thing pressed
        
        match self.controller.getPOV():
            case 0:
                self.arm_angle += 3
            case 180:
                self.arm_angle -= 3
            
             #used for testing 
            case 270:
                self.l_arm.set(0.3 * self.arm_ratio)
                self.r_arm.set(-0.3)
            case 90:
                self.l_arm.set(-0.3 * self.arm_ratio)
                self.r_arm.set(0.3)
           
            case _:
                # might be redundant
                self.l_arm.set(0)
                self.r_arm.set(0)



     # PID Arm Control
                
        if self.controller.getAButton():
            self.arm_angle = 5
        elif self.controller.getBButton():
            self.arm_angle = 90
        elif self.controller.getYButton():
            self.arm_angle = 30
        # Arm P
        self.arm_sensor_position = self.arm_encoder.getPosition() * self.arm_tick_constant
        self.arm_error = self.arm_angle - self.arm_sensor_position
        
        print('Arm Sensor Position: ') 
        print(self.arm_sensor_position)
        print('Kp Constant: ')
        print(self.arm_Kp)
        #Arm I
        self.arm_dt = wpilib.getTime() - self.arm_last_timestamp
        # I Zone
        if abs(self.arm_error) < self.arm_KiZone:
            self.arm_error_sum += self.arm_error * self.arm_dt
        #Arm D
        self.arm_error_rate = (self.arm_error - self.arm_last_error) / self.arm_dt

        #Find PID Speed Control
        self.arm_speed = (self.arm_Kp * self.arm_error + self.arm_Ki * self.arm_error_sum + self.arm_Kd * self.arm_error_rate)/100
        print('Arm Speed: ')
        print(self.arm_speed)

        # THE ARMS DON'T MOVE AT THE SAME SPEED UGHHHH!!!! THERE IS A GEAR RATIO CONSTANT OF 1.25 APPLIED TO THE RIGHT ARM
        # BUT IT DOESNT WORK! CAREFUL BECAUSE THEY REALLY WANT TO TEAR THEMSELVES IN HALF
        self.l_arm.set(1 * self.arm_speed)
        self.r_arm.set(1 * self.arm_speed)

        #updating PID varibles
        self.arm_last_timestamp = wpilib.getTime()
        self.arm_last_error = self.arm_error

               
        # self.l_arm.set(self.arm_controller.calculate(self.arm_encoder.getPosition(), ))
        