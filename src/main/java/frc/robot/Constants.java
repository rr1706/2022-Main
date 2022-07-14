package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utilities.LinearInterpolationTable;

import java.awt.geom.Point2D;

/**
 * Static method containing all constant values for the robot in one location
 */
public final class Constants {

  public static final class CurrentLimit {
    public static final int kIntake = 25;
    public static final int kTurret = 20;
    public static final int kShooter = 40;
    public static final int kHood = 20;
    public static final int kElevator = 30;
    public static final int kTranslation = 30;
    public static final int kRotation = 25;
    public static final int kClimber = 80;
  }

  public static final class GoalConstants {
    public static final Translation2d kGoalLocation = new Translation2d(8.23, 4.115);
    public static final Translation2d kWrongBallGoal = new Translation2d(5.50, 4.115);
    public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);

  }

  /**
   * Static method containing all Drivetrain constants
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 19; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 4; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 6; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 2; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 18; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 3; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 7; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 1; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -1.1634 - 0.269 - 0.346 + 3.141593; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -3.442 + 3.1415; // Encoder Offset in Radians
    public static final double kBackLeftOffset = -0.0884 - 5.55 + 2.816 + 3.141593; // Encoder Offset in Radians
    public static final double kBackRightOffset = -1.6194 + 0.542 + 3.14159 - 7.985; // Encoder Offset in Radians

    // Drive motor PID is best done on the roboRIO currently as the SparkMAX does
    // not allow for static gain values on the PID controller,
    // these are necessary to have high accuracy when moving at extremely low RPMs
    // public static final double[] kFrontLeftTuningVals = {0.0120,0.2892,0.25,0};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kFrontRightTuningVals = {0.0092,0.2835,0.25,1};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackLeftTuningVals = {0.0142,0.2901,0.25,2};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackRightTuningVals = {0.0108,0.2828,0.25,3};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals = { 0.0150, 0.2850, 0.25, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150, 0.2850, 0.25, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150, 0.2850, 0.25, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150, 0.2850, 0.25, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.6446; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)
    public static final double kTranslationSlew = 1.45;
    public static final double kRotationSlew = 3.00;

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    public static final double[] kKeepAnglePID = { 0.500, 0, 0 }; // Defines the PID values for the keep angle PID

  }

  /**
   * Static method containing all Swerve Module constants
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                           // 100% power from 0%
    private static final double kTranslationGearRatio = 8.33333333; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.0986 * 0.960 * 0.995; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                  // the
                                                                                                                  // conversion
                                                                                                                  // factor
                                                                                                                  // of
                                                                                                                  // RPM
                                                                                                                  // of
                                                                                                                  // the
                                                                                                                  // translation
                                                                                                                  // motor
                                                                                                                  // to
                                                                                                                  // m/s
                                                                                                                  // at
                                                                                                                  // the
                                                                                                                  // floor

    // NOTE: You shoulds ALWAYS define a reasonable current limit when using
    // brushless motors
    // due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  /**
   * Static method containing all User I/O constants
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // When making use of multiple controllers for drivers each
                                                       // controller will be on a different port
    public static final int kOperatorControllerPort = 1; // When making use of multiple controllers for drivers each
                                                         // controller will be on a different port
  }

  /**
   * Static method containing all Global constants
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  /**
   * Static method containing all Vision/Limelight constants
   */
  public static final class VisionConstants {
    public static final double kElevationOffset = 38.5; // Degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = 0.0; // Degree azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0; // Center Height of the Target in inches above the
                                                                   // lens
    public static final double kTrackTolerance = 0.0200; // Allowable Limelight angle error in radians
  }

  /**
   * Static method containing all Intake constants
   */
  public static final class IntakeConstants {
    public static final int kLeftMotorID = 8;
    public static final int kRightMotorID = 5;
    public static final int[] kLeftAirPorts = { 3, 2 };
    public static final int[] kRightAirPorts = { 1, 0 };
    public static final double kP = 0.0001;
    public static final double kFF = 0.0000970;
    public static final double kSpeed = 11000.0;
    public static final double kRevSpeed = -3000.0;
  }

  public static final class ElevatorConstants {
    public static final int kLowMotorID = 17;
    public static final int kHighMotorID = 16;
    public static final int kLowSensor = 10;
    public static final int kHighSensor = 11;

    public static final double kP = 0.0001;
    public static final double kFF = 0.000097;

    public static final double kSpeed = 10000.0;
  }

  public static final class TurretConstants {
    public static final int kPort = 14; // CANID of the turret motor controller
    public static final double kTolerance = 2 * 0.0349; // allowable angle error in radians for the PIDSubsystem to
                                                        // report atSetpoint() to true
    public static final double kP = 0.0; // Defines the PID values for rotation of the turret
    public static final double kFF = 0.00017; // Limits Integral term so as to not wind up values when making larger
                                              // moves
    public static final double kLow = 0.383; // Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kHigh = 5.90; // Maximum angle in radians allowed (defines the turret deadzone)
    public static final double kAccel = 15000;
    public static final double kMaxVel = 5000;
    public static final double kMotionTol = 0.2;
    public static final double kRatio = 0.0171875;
    public static final double kStartingPosition = 1 / kRatio / 2;
    public static final float kLowLimit = (float) (kLow / kRatio / 2.0 / Math.PI - 1.0);
    public static final float kHighLimit = (float) (kHigh / kRatio / 2.0 / Math.PI + 1.0);
    public static final double kNearDeadzone = 0.20;
  }

  public static final class ClimberConstants {
    public static final int[] kMotorID = { 15, 9 };
    public static final int[] kValvePorts = { 4, 5 };
    public static final double[] kPID = { 0.25, 0, 0 };
    public static final Constraints kDefaultConstraints = new Constraints(50, 25);
    public static final double kMinPose = -15.0;
    public static final double kMaxPose = 72.0;
    public static final double kTolerance = 1.0;
    public static final double kExtendPose = 69.5;
  }

  public static final class HoodConstants {
    public static final int kMotorID = 11;
    public static final double kHoodTolerance = 5.0;
    public static final double kMaxAngle = 38.0;
    public static final double kMinAngle = 0.5;
    public static final double kPosConvFactor = 1.27777777778;

    public static final double kP = 0.20;
    public static final double kMaxNegPower = -0.33;
    public static final double kMaxPosPower = 1.0;
  }

  public static final class ShooterConstants {
    public static final int[] kMotorIDs = { 12, 13 }; // CANID of the Motor Controller for the Sooter Motor
    public static final double kRPMTolerance = 200.0; // RPMs of error allowed before a ball can be fed into t he
                                                      // shooter
    public static final double[] kPID = { 0.00005, 0.0003, 0 }; // Defines PID values for the shooter 0.00045
    public static final double kIntRange = 0.015;
    public static final double kStatic = 0.018;
    public static final double kFF = 0.0001635;
    public static final double kAccelCompFactor = 0.100; // in units of seconds
    public static final double kMaxRPM = 4000.0;
    public static final double kMaxNegPower = -0.25;

    public static final double kHangarRPM = 1200;

    private static final Point2D[] kHoodPoints = new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(25, 0.0),
        new Point2D.Double(35, 0.0),
        new Point2D.Double(75, 10.0), //
        new Point2D.Double(90, 15.5), //
        new Point2D.Double(105, 18.4), //
        new Point2D.Double(120, 23.0), //
        new Point2D.Double(135, 26.5), //
        new Point2D.Double(150, 29.0), //
        new Point2D.Double(165, 30.5), //
        new Point2D.Double(180, 32.0), //
        new Point2D.Double(195, 36.0), //
        new Point2D.Double(210, 37.0), //
        new Point2D.Double(225, 38.0)//
    };
    public static final LinearInterpolationTable kHoodTable = new LinearInterpolationTable(kHoodPoints);

    private static final Point2D[] kRPMPoints = new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(25, 1500),
        new Point2D.Double(35, 1500),
        new Point2D.Double(75, 2340), //
        new Point2D.Double(90, 2450), //
        new Point2D.Double(105, 2505), //
        new Point2D.Double(120, 2610), //
        new Point2D.Double(135, 2710), //
        new Point2D.Double(150, 2815), //
        new Point2D.Double(165, 2890), //
        new Point2D.Double(180, 3040), //
        new Point2D.Double(195, 3185), //
        new Point2D.Double(210, 3315), //
        new Point2D.Double(225, 3450), //
        new Point2D.Double(350, 3800)
    };

    public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

    private static final Point2D[] kShotTimes = new Point2D.Double[] {
        // (ty-angle,time)
        new Point2D.Double(105, 0.82),
        new Point2D.Double(135, 0.82),
        new Point2D.Double(165, 0.85),
        new Point2D.Double(195, 0.85),
        new Point2D.Double(250, 1.05),
    };

    public static final LinearInterpolationTable kTimeTable = new LinearInterpolationTable(kShotTimes);
  }

  public static final class ColorConstants{
    public static final Color kBlueTarget = new Color(0.215, 0.434, 0.350);
    public static final Color kRedTarget = new Color(0.335, 0.421, 0.242);
    public static final Color kNoTarget = new Color(0.256, 0.457, 0.287);
  }

  /**
   * Static method containing all Autonomous constants
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 2.50;
    public static final double kMaxSpeed = 3.25; // Maximum Sustainable Drivetrain Speed under Normal Conditions &
                                                 // Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel); // Creates a trapezoidal motion for the auto rotational commands
  }
}