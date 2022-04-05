package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities.MathUtils;

public class Turret extends PIDSubsystem {
    private final AnalogPotentiometer m_encoder = new AnalogPotentiometer(4, 2*Math.PI*1.011, -0.007636);
    private final CANSparkMax m_motor = new CANSparkMax(14, MotorType.kBrushed);
    private boolean m_trackTarget = false;

    public Turret(){
        super(new PIDController(
            TurretConstants.kTurretPID[0], TurretConstants.kTurretPID[1], TurretConstants.kTurretPID[2]));
            m_controller.disableContinuousInput();
            m_controller.setSetpoint(Math.PI);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(CurrentLimit.kTurret);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.burnFlash();
        m_controller.setTolerance(TurretConstants.kTurretTolerance);
        SmartDashboard.putBoolean("EnableLimelight", false);
    }

    public double getMeasurement(){
        double angle = MathUtils.toUnitCircAngle(-1.0*m_encoder.get())+0.1544-0.0244;
       // SmartDashboard.putNumber("Turret Encoder", angle);
        return angle;
    }

/*     @Override 
    public void periodic(){
      SmartDashboard.putNumber("Turret Setpoint", m_controller.getSetpoint());

    } */

    public void aimAtGoal(Pose2d robotPose, Translation2d goal){
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

      if (angle < TurretConstants.kTurretLow) {
        angle = TurretConstants.kTurretLow;
      } 
      //if the angle setpoint is hgiher than the maximum allowed position, set the setpoint to the maximum allowed position
      //and set the turret to search counter clockwise 
      else if (angle > TurretConstants.kTurretHigh) {
        angle = TurretConstants.kTurretHigh;
      } 
      else{
        setSetpoint(angle);
      }
    }

    public void setAngle(Pose2d robotPose){

    
        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(robotPose.getTranslation());

        //double dist = robotToGoal.getDistance(new Translation2d());
        //SmartDashboard.putNumber("Distance to Goal (m)", dist);
        //SmartDashboard.putNumber("Distance to Goal (in)", dist*39.37);
    
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
      else{
        setSetpoint(angle);
      }

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

    @Override
    protected void useOutput(double output, double setpoint) {
        if(getMeasurement()>TurretConstants.kTurretHigh && output>0){
            m_motor.set(-0.2);
        }
        else if(getMeasurement()<TurretConstants.kTurretLow && output<0){
            m_motor.set(0.2);
        }
        else{
            m_motor.set(output);
        }
    }

    public void stop(){
      m_motor.stopMotor();
    }

    public boolean atSetpoint(){
        return m_controller.atSetpoint();
    }
    public void climbMode(){
        setSetpoint(Math.PI);
    }

}


