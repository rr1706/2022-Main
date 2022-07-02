package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    private final CANSparkMax m_hoodMotor = new CANSparkMax(HoodConstants.kMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_hoodEncoder = m_hoodMotor.getEncoder();
    private final SparkMaxPIDController m_hoodPID = m_hoodMotor.getPIDController();

    //Maybe try using SmartMotion on the SparkMax at some point??????????

    private double m_hoodAngle = HoodConstants.kMinAngle;

    public ShooterHood() {
        m_hoodMotor.setSmartCurrentLimit(CurrentLimit.kHood);
        m_hoodMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        m_hoodEncoder.setPositionConversionFactor(HoodConstants.kPosConvFactor);
        m_hoodPID.setP(HoodConstants.kP);
        m_hoodPID.setOutputRange(HoodConstants.kMaxNegPower, HoodConstants.kMaxPosPower);
        m_hoodMotor.setInverted(true);
        m_hoodEncoder.setPosition(0.0);
        m_hoodMotor.burnFlash();
    }

    public void run(double angle) {
        setHoodAngle(angle);
        m_hoodPID.setReference(m_hoodAngle, ControlType.kPosition);
    }

    public void setHoodPower(double power) {
        m_hoodMotor.set(power);
    }

    public double getTotalCurrent() {
        return m_hoodMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Hood Angle", getHoodAngle());

    }

    public void setHoodAngle(double angle) {
        m_hoodAngle = angle;
        if (m_hoodAngle < HoodConstants.kMinAngle) {
            m_hoodAngle = HoodConstants.kMinAngle;
        } else if (m_hoodAngle > HoodConstants.kMaxAngle) {
            m_hoodAngle = HoodConstants.kMaxAngle;
        }
    }

    public void setHoodZero() {
        m_hoodEncoder.setPosition(0.0);
        m_hoodMotor.stopMotor();
    }

    /*
     * public boolean getHoodLimit(){
     * return m_hoodLimit.get();
     * }
     */

    public double getHoodAngle() {
        return m_hoodEncoder.getPosition();
    }

    public void stop() {
        m_hoodMotor.stopMotor();
    }

    public boolean atSetpoint() {
        return Math.abs(m_hoodAngle - getHoodAngle()) < HoodConstants.kHoodTolerance;
    }

}
