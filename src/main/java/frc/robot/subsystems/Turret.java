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
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities.MathUtils;

public class Turret extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(14, MotorType.kBrushless);
    private final SparkMaxPIDController m_controller = m_motor.getPIDController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    private boolean m_trackTarget = false;
    private double m_desiredAngle = 0.0;

    public Turret(){
        m_controller.setP(0.00000,0);
        m_controller.setFF(0.00017,0);
        m_controller.setSmartMotionMaxAccel(15000.0,0);
        m_controller.setSmartMotionMaxVelocity(5000.0, 0);
        m_controller.setSmartMotionAllowedClosedLoopError(0.2, 0);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)55.7575757);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse,  (float)2.4242424);
        m_motor.setSmartCurrentLimit(CurrentLimit.kTurret);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_encoder.setPosition(29.090909);
        m_motor.burnFlash();

        SmartDashboard.putBoolean("EnableLimelight", false);
    }

    public double getMeasurement(){
        double angle = m_encoder.getPosition()*0.0171875*2*Math.PI;
        return angle;
    }

    @Override 
    public void periodic(){
      SmartDashboard.putNumber("Turret Location", getMeasurement());

    } 

    public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision){
      Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
      double angle = Math.atan2(robotToGoal.getY(),robotToGoal.getX());
      angle = Math.PI+angle-robotPose.getRotation().getRadians();

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


      if (angle < TurretConstants.kTurretLow) {
        angle = TurretConstants.kTurretLow;
      } 
      //if the angle setpoint is hgiher than the maximum allowed position, set the setpoint to the maximum allowed position
      //and set the turret to search counter clockwise 
      else if (angle > TurretConstants.kTurretHigh) {
        angle = TurretConstants.kTurretHigh;
      } 

      double neoRevs = angle*58.1818181/(2*Math.PI); 

      m_controller.setReference(neoRevs, ControlType.kSmartMotion);
    }

    public void setAngle(Pose2d robotPose){

    
        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(robotPose.getTranslation());
    
        double angle = Math.atan2(robotToGoal.getY(),robotToGoal.getX());

        angle = Math.PI+angle-robotPose.getRotation().getRadians();

        SmartDashboard.putNumber("Turret Set Angle", angle);

        angle = MathUtils.toUnitCircAngle(angle);

        if (m_trackTarget || SmartDashboard.getBoolean("EnableLimelight", false)) {
            Limelight.enable();
          } else {
            Limelight.disable();
          }

   //When the Limelight has a valid solution , use the limelight tx() angle and add it to the current turret postiion to 
    //determine the updated setpoint for the turret
     if (m_trackTarget && Limelight.valid()) {
        angle = getMeasurement() - Limelight.tx();
      } 

      m_desiredAngle = angle;

      //if the angle setpoint is lower than the minimum allowed position, set the setpoint to the minimum allowed position
      //and set the turret to search clockwise 
      if (angle < TurretConstants.kTurretLow) {
        angle = TurretConstants.kTurretLow;
      } 
      //if the angle setpoint is hgiher than the maximum allowed position, set the setpoint to the maximum allowed position
      //and set the turret to search counter clockwise 
      else if (angle > TurretConstants.kTurretHigh) {
        angle = TurretConstants.kTurretHigh;
      } 
      
      double neoRevs = angle*58.1818181/(2*Math.PI); 

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

    /**
   * Because the same PIDController is used for the turret at all times, a simple atSetpoint() call is not good enough to know it is
   * okay to shoot. Instead this fucntion is defined for when there is a valid solution and the limelight value is within the allowable
   * tolerance
   */

  public boolean visionAligned() {
    if (Limelight.valid() && Math.abs(Limelight.tx()) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }


    public void stop(){
      m_motor.stopMotor();
    }

    public boolean atDesiredAngle(){
      return Math.abs(m_desiredAngle-getMeasurement()) <= TurretConstants.kTurretTolerance;
    }

    public boolean desiredInDeadzone(){
      return (m_desiredAngle >= TurretConstants.kTurretHigh-0.20 || m_desiredAngle <= TurretConstants.kTurretLow+0.20);
    }

    public void climbMode(){
        m_controller.setReference(29.0909091, ControlType.kSmartMotion);
    }

    public void disable() {
      m_controller.setReference(29.0909091, ControlType.kSmartMotion);
    }

    public void enable() {
      m_controller.setReference(29.0909091, ControlType.kSmartMotion);
    }

}


