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

public class Shooter extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    private final PIDController m_PID = new PIDController(ShooterConstants.kPID[0], ShooterConstants.kPID[1],
            ShooterConstants.kPID[2]);
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(ShooterConstants.kStatic, ShooterConstants.kFF);

    private double m_RPM = ShooterConstants.kMaxRPM;

    public Shooter(int motorIDs[]) {

        m_motor1 = new CANSparkMax(motorIDs[0], MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motorIDs[1], MotorType.kBrushless);
        m_encoder1 = m_motor1.getEncoder();
        m_encoder2 = m_motor2.getEncoder();

        m_motor1.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor2.setSmartCurrentLimit(CurrentLimit.kShooter);
        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);

        m_motor1.setInverted(false);
        m_motor2.follow(m_motor1, true);

        m_encoder1.setVelocityConversionFactor(1.0);

        m_motor1.burnFlash();
        m_motor2.burnFlash();

        m_PID.setTolerance(ShooterConstants.kRPMTolerance);
        m_PID.setIntegratorRange(-ShooterConstants.kIntRange, ShooterConstants.kIntRange);

    }

    public void run(double rpm) {
        if (rpm >= ShooterConstants.kMaxRPM) {
            rpm = ShooterConstants.kMaxRPM;
        }
        m_RPM = rpm;
        double outputPID = m_PID.calculate(m_encoder1.getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        double output = outputPID + outputFF;

        if (output <= ShooterConstants.kMaxDecel) {
            output = ShooterConstants.kMaxDecel;
        }

        m_motor1.set(outputPID + outputFF);
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", m_encoder1.getVelocity());

    }

    public boolean atSetpoint() {
        return m_PID.atSetpoint();
    }

}
