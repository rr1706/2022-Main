package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities.MathUtils;

public class Turret extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(14, MotorType.kBrushless);
  private final SparkMaxPIDController m_controller = m_motor.getPIDController();
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private boolean m_trackTarget = false;
  private double m_desiredAngle = 0.0;

  public Turret() {
    m_controller.setP(TurretConstants.kP, 0);
    m_controller.setFF(TurretConstants.kFF, 0);
    m_controller.setSmartMotionMaxAccel(TurretConstants.kAccel, 0);
    m_controller.setSmartMotionMaxVelocity(TurretConstants.kMaxVel, 0);
    m_controller.setSmartMotionAllowedClosedLoopError(TurretConstants.kMotionTol, 0);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, TurretConstants.kHighLimit);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, TurretConstants.kLowLimit);
    m_motor.setSmartCurrentLimit(CurrentLimit.kTurret);
    m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    m_encoder.setPosition(TurretConstants.kStartingPosition);
    m_motor.burnFlash();

    SmartDashboard.putBoolean("EnableLimelight", false);
  }

  public double getMeasurement() {
    double angle = m_encoder.getPosition() * TurretConstants.kRatio * 2 * Math.PI;
    return angle;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Location", getMeasurement());

  }

  public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision) {
    Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
    double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());
    angle = Math.PI + angle - robotPose.getRotation().getRadians();

    angle = MathUtils.toUnitCircAngle(angle);

    if (m_trackTarget) {
      Limelight.enable();
    } else {
      Limelight.disable();
    }

    SmartDashboard.putNumber("Turret Set Angle", angle);

    if (aimAtVision && Limelight.valid()) {
      angle = getMeasurement() - Limelight.tx();
    }

    m_desiredAngle = angle;

    if (angle < TurretConstants.kLow) {
      angle = TurretConstants.kLow;
    } else if (angle > TurretConstants.kHigh) {
      angle = TurretConstants.kHigh;
    }

    double neoRevs = angle / TurretConstants.kRatio / (2 * Math.PI);

    m_controller.setReference(neoRevs, ControlType.kSmartMotion);
  }

  /**
   * Function to set the track target boolean to either true or false
   * 
   * @param track is true when Limelight tracking is desired
   */
  public void trackTarget(boolean track) {
    m_trackTarget = track;
  }

  public boolean visionAligned() {
    if (Limelight.valid() && Math.abs(Limelight.tx()) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public boolean atDesiredAngle() {
    return Math.abs(m_desiredAngle - getMeasurement()) <= TurretConstants.kTolerance;
  }

  public boolean closeToDeadzone() {
    return (m_desiredAngle >= TurretConstants.kHigh - TurretConstants.kNearDeadzone
        || m_desiredAngle <= TurretConstants.kLow + TurretConstants.kNearDeadzone);
  }

  public void setToStartPosition() {
    m_desiredAngle = TurretConstants.kStartingPosition*TurretConstants.kRatio*2*Math.PI;
    m_controller.setReference(TurretConstants.kStartingPosition, ControlType.kSmartMotion);
  }

  public boolean readyToClimb() {
    return Math.abs(m_desiredAngle - getMeasurement()) <= Math.PI/2.0;
  }

}
