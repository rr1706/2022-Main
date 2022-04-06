package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utilities.LinearInterpolationTable;

import java.awt.geom.Point2D;

  /**
   * Static method containing all constant values for the robot in one location
   */
public final class Constants {

  public static final class CurrentLimit{
    public static final int kIntake = 25;
    public static final int kTurret = 30;
    public static final int kShooter = 40;
    public static final int kHood = 20;
    public static final int kElevator = 30;
    public static final int kTranslation = 30;
    public static final int kRotation = 25;
    public static final int kClimber = 80;
  }

  public static final class GoalConstants{
    public static final Translation2d kGoalLocation = new Translation2d(8.23,4.115);
    public static final Translation2d kHangerLocation = new Translation2d(1.67,6.70);

  }

  /**
   * Static method containing all Drivetrain constants 
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 19;   //CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 4;  //CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 6;    //CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 2;   //CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 18;   //CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 3;  //CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 7;    //CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 1;   //CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0;   //Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1;  //Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 3;    //Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 2;   //Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -1.1634-0.269-0.346+3.141593;  //Encoder Offset in Radians
    public static final double kFrontRightOffset = -3.442 +3.1415;  //Encoder Offset in Radians
    public static final double kBackLeftOffset = -0.0884-5.55+2.816+3.141593;   //Encoder Offset in Radians
    public static final double kBackRightOffset = -1.6194+0.542+3.14159-7.985;  //Encoder Offset in Radians

    //Drive motor PID is best done on the roboRIO currently as the SparkMAX does not allow for static gain values on the PID controller, 
    //    these are necessary to have high accuracy when moving at extremely low RPMs
    //public static final double[] kFrontLeftTuningVals   =   {0.0120,0.2892,0.25,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kFrontRightTuningVals  =   {0.0092,0.2835,0.25,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackLeftTuningVals    =   {0.0142,0.2901,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackRightTuningVals   =   {0.0108,0.2828,0.25,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals   =   {0.0150,0.2850,0.25,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kFrontRightTuningVals  =   {0.0150,0.2850,0.25,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackLeftTuningVals    =   {0.0150,0.2850,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackRightTuningVals   =   {0.0150,0.2850,0.25,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}


    //NOTE: 2910 Swerve the wheels are not directly under the center of rotation (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588;  // Center distance in meters between right and left wheels on robot
    public static final double kWheelBaseLength = 0.6446;   // Center distance in meters between front and back wheels on robot
     
    //Because the swerve modules poisition does not change, define a constant SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2), new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2), new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 3.25; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kInnerDeadband = 0.10; //This value should exceed the maximum value the analog stick may read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; //This value should be lower than the analog stick X or Y reading when aimed at a 45deg angle (Such that X and Y are are maximized simultaneously)
  
    //Minimum allowable rotation command (in radians/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed * Math.pow(DriveConstants.kInnerDeadband,2);
    //Minimum allowable tranlsation command (in m/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond * Math.pow(DriveConstants.kInnerDeadband,2);

    public static final double[] kKeepAnglePID = { 0.500, 0, 0 }; //Defines the PID values for the keep angle PID

  }
  /**
   * Static method containing all Swerve Module constants 
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0;          //Units of %power/s, ie 4.0 means it takes 0.25s to reach 100% power from 0%
    private static final double kTranslationGearRatio = 8.33333333; //Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0986*0.960*0.995;           //Wheel Diameter in meters, may need to be experimentally determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; //Calculates the conversion factor of RPM of the translation motor to m/s at the floor

    //NOTE: You shoulds ALWAYS define a reasonable current limit when using brushless motors 
    //      due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; //Defines the PID values for rotation of the serve modules, should show some minor oscillation when no weight is loaded on the modules
  }
  /**
   * Static method containing all User I/O constants 
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;    //When making use of multiple controllers for drivers each controller will be on a different port
    public static final int kOperatorControllerPort = 1;  //When making use of multiple controllers for drivers each controller will be on a different port
  }

  /**
   * Static method containing all Global constants 
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6;        //Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
  /**
   * Static method containing all Vision/Limelight constants 
   */
  public static final class VisionConstants {
    public static final double kElevationOffset =38.5;              // Degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = -0.50;                // Degree azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0;  // Center Height of the Target in inches above the lens
    public static final double kTrackTolerance = 0.0200;             // Allowable Limelight angle error in radians
  }
  /**
   * Static method containing all Intake constants 
   */
  public static final class IntakeConstants {
    public static final int kLeftMotorID = 8;
    public static final int kRightMotorID = 5;
    public static final int[] kLeftAirPorts = {3,2};
    public static final int[] kRightAirPorts = {1,0};
    public static final double []kPIDF = {0.00005,0,0,0.000091};
  }

  public static final class ElevatorConstants {
    public static final int kLowMotorID = 17;
    public static final int kHighMotorID = 16;
    public static final double []kPIDF = {0.000075,0,0,0.000091};
    public static final int kLowSensor = 10;
    public static final int kHighSensor = 11;
  }

  public static final class TurretConstants {
    public static final int kTurretPort = 14;                    //CANID of the turret motor controller
    public static final int kTurretPotentiometerPort = 4;       //Analog port of the turret analog potentiometer
    public static final double kTurretTolerance = 0.01745;    //allowable angle error in radians for the PIDSubsystem to report atSetpoint() to true
    public static final double[] kTurretPID = { 4.0, 0.0, 0 };  //Defines the PID values for rotation of the turret
    public static final double kStaticGain = 0.025;             //Limits Integral term so as to not wind up values when making larger moves
    public static final double kTurretLow = 0.26;               //Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kTurretHigh = 6.02;              //Maximum angle in radians allowed (defines the turret deadzone)
  }

  public static final class ClimberConstants {
    public static final int[] kMotorID = {15,9};
    public static final int[] kValvePorts = {4,5};
  }

  public static final class HoodConstants{
    public static final int kMotorID = 11;
    public static final double kHoodTolerance = 1.0;
  }

  /**
   * Static method containing all Shooter constants 
   */
  public static final class ShooterConstants {
    public static final int[] kMotorIDs = {12,13};        //CANID of the Motor Controller for the Sooter Motor
    public static final double kShotRPMTolerance = 50.0;          //RPMs of error allowed before a ball can be fed into t he shooter
    public static final double[] kPID = { 0.00005, 0.0003, 0 };         //Defines PID values for the shooter 0.00045
    public static final double kShooterFF = 0.018;            //Defines shooter FeedForward Value, should be roughly equal to 1/MaxMotorRPM * MaxRPMVoltage / Compensation Voltage
    public static final double kStaticGain = 0.0001635;
    public static final double kAccelCompFactor = 0.100; //in units of seconds


    private static final Point2D[] khoodPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,0.0),
            new Point2D.Double(50,0.0),
            new Point2D.Double(75,10.0),//
            new Point2D.Double(90,15.5),//
            new Point2D.Double(105,19.0),//
            new Point2D.Double(120,23.0),//
            new Point2D.Double(135,25.5),//
            new Point2D.Double(150,27.5),//
            new Point2D.Double(165,29.5),//
            new Point2D.Double(180,33.0),//
            new Point2D.Double(195,36.0),//
            new Point2D.Double(210,38.0),//
            new Point2D.Double(240,38.0)//
        };
    public static final LinearInterpolationTable khoodTable = new LinearInterpolationTable(khoodPoints);

    private static final Point2D[] krpmPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,2350),
            new Point2D.Double(50,2350),
            new Point2D.Double(75,2375),//
            new Point2D.Double(90,2480),//
            new Point2D.Double(105,2510),//
            new Point2D.Double(120,2610),//
            new Point2D.Double(135,2695),//
            new Point2D.Double(150,2785),//
            new Point2D.Double(165,2885),//
            new Point2D.Double(180,3045),//
            new Point2D.Double(195,3180),//
            new Point2D.Double(210,3315),//
            new Point2D.Double(240,3500),//
            new Point2D.Double(280,3850),
        };

    public static final LinearInterpolationTable krpmTable = new LinearInterpolationTable(krpmPoints);
  }
  
    /**
   * Static method containing all Autonomous constants 
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.25; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel); //Creates a trapezoidal motion for the auto rotational commands
  }
}