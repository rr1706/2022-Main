package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    private final PIDController m_PID = new PIDController(0.00005, 0.0003, 0.0);
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(0.018, 0.0001635);
    
    private double m_RPM = 3000.0;

    public Shooter(int motorIDs[]) {

        m_motor1 = new CANSparkMax(motorIDs[0],MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motorIDs[1],MotorType.kBrushless);
        m_encoder1 = m_motor1.getEncoder();
        m_encoder2 = m_motor2.getEncoder();

        m_motor1.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor2.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor1.setInverted(false);
        m_motor2.follow(m_motor1,true);

        m_encoder1.setVelocityConversionFactor(1.0);

        m_motor1.burnFlash();
        m_motor2.burnFlash();

        m_PID.setTolerance(ShooterConstants.kShotRPMTolerance);
        m_PID.setIntegratorRange(-0.015, 0.015);

        //SmartDashboard.putNumber("SetShotRPM", m_RPM);
       // SmartDashboard.putBoolean("Shooter PID Reset", false);
        //SmartDashboard.putNumber("Shooter P", m_PID.getP());
        //SmartDashboard.putNumber("Shooter I", m_PID.getI());
        //SmartDashboard.putNumber("Shooter FF", m_FF.kv);
        //SmartDashboard.putNumber("Shooter Static", m_FF.ks);
        //SmartDashboard.putNumber("Shooter Max I Power", 0.02);

    }

    public void run(double rpm) {
        m_RPM = rpm;
        double outputPID = m_PID.calculate(m_encoder1.getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        m_motor1.set(outputPID+outputFF);
    }


    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", m_encoder1.getVelocity());
       // SmartDashboard.putNumber("Shooter RPM2", m_encoder2.getVelocity());
        //SmartDashboard.putNumber("Shooter Current", m_motor1.getOutputCurrent()+m_motor2.getOutputCurrent());
        //m_RPM = SmartDashboard.getNumber("SetShotRPM", 3000.0);

/* 
        if(SmartDashboard.getBoolean("Shooter PID Reset", false)){
            double kv = SmartDashboard.getNumber("Shooter FF", 0.00016);
            double ks = SmartDashboard.getNumber("Shooter Static", 0.02);
            double integratePower = SmartDashboard.getNumber("Shooter Max I Power", 0.02);
            m_PID.setP(SmartDashboard.getNumber("Shooter P", 0.0001));
            m_PID.setI(SmartDashboard.getNumber("Shooter I", 0.0001));
            m_PID.setIntegratorRange(-integratePower, integratePower);
            m_FF = new SimpleMotorFeedforward(ks, kv);
            SmartDashboard.putBoolean("Shooter PID Reset", false);
        } */




    }

    public boolean atSetpoint() {
        return m_PID.atSetpoint();
    }
    
}
