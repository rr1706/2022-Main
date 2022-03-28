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
import frc.robot.Utilities.LinearInterpolationTable;

import java.awt.geom.Point2D;

public class ShooterHood extends SubsystemBase {
    private final CANSparkMax m_hoodMotor = new CANSparkMax(HoodConstants.kMotorID,MotorType.kBrushless);
    private final RelativeEncoder m_hoodEncoder = m_hoodMotor.getEncoder();
    private final SparkMaxPIDController m_hoodPID = m_hoodMotor.getPIDController();
    //private final DigitalInput m_hoodLimit = new DigitalInput(10);

    private static Point2D[] points = 
    new Point2D.Double[]{
        //(ty-angle,distance)
        new Point2D.Double(-9.95,267.5),
        new Point2D.Double(-5.03,205.7),
        new Point2D.Double(0.05,165.8),
        new Point2D.Double(5.02,141.5),
        new Point2D.Double(10.03,122.5),
        new Point2D.Double(15.01,108),
        new Point2D.Double(19.99,99),
        new Point2D.Double(23.25,89)
    };
private static LinearInterpolationTable angleTable = new LinearInterpolationTable(points);


    private double m_hoodAngle = 15.0;

    public ShooterHood(){
        m_hoodMotor.setSmartCurrentLimit(CurrentLimit.kHood);
        m_hoodMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        m_hoodEncoder.setPositionConversionFactor(1.2777778);
        m_hoodPID.setP(0.20);
        m_hoodPID.setI(0.0);
        m_hoodPID.setIMaxAccum(0.0003, 0);
        m_hoodPID.setD(0.0);
        m_hoodPID.setFF(0.00);
        m_hoodPID.setOutputRange(-0.33, 1.0);
        m_hoodMotor.setInverted(true);
        m_hoodMotor.burnFlash();

        //SmartDashboard.putNumber("SetHoodAngle", m_hoodAngle);
        m_hoodEncoder.setPosition(0.0);
    }

    public void run(double angle) {
        setHoodAngle(angle);
        //setHoodAngle(SmartDashboard.getNumber("SetHoodAngle", 10));
        m_hoodPID.setReference(m_hoodAngle, ControlType.kPosition);
    }

    public void setHood(double speed){
        m_hoodMotor.set(speed);
    }

    public double getTotalCurrent(){
        return m_hoodMotor.getOutputCurrent();
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Hood Current", getTotalCurrent());
        SmartDashboard.putNumber("Current Hood Angle", getHoodAngle());

    }

    public void setHoodAngle(double angle){
        m_hoodAngle = angle;
        if(m_hoodAngle < 0.0){
            m_hoodAngle = 0.0;
        }
        else if(m_hoodAngle > 38.0){
            m_hoodAngle = 38.0;
        }
    }

    public void setHoodZero(){
        m_hoodEncoder.setPosition(0.0);
        m_hoodMotor.stopMotor();
    }

    /* public boolean getHoodLimit(){
        return m_hoodLimit.get();
    } */

    public double getHoodAngle(){
        return m_hoodEncoder.getPosition();
    }

    public void stop() {
        m_hoodMotor.stopMotor();
    }
    public boolean atSetpoint(){
        return Math.abs(m_hoodAngle-getHoodAngle())<HoodConstants.kHoodTolerance;
    }
    
}
