package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_PID;
    private final DoubleSolenoid m_actuator;
    private final String m_ID;

    public Intake(int motorCANID, int[] airChannels, String ID) {
        m_ID = ID;
        m_motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_PID = m_motor.getPIDController();
        m_actuator = new DoubleSolenoid(GlobalConstants.PCHID, PneumaticsModuleType.REVPH, airChannels[0],
                airChannels[1]);

        m_motor.setSmartCurrentLimit(CurrentLimit.kIntake);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_PID.setOutputRange(-1.0, 1.0);
        m_PID.setP(IntakeConstants.kP);
        m_PID.setFF(IntakeConstants.kFF);
        m_motor.burnFlash();

    }

    public void run(double rpm) {
        m_PID.setReference(rpm, ControlType.kVelocity);
    }

    public void extend() {
        m_actuator.set(Value.kForward);
    }

    public void retract() {
        m_actuator.set(Value.kReverse);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public String getID() {
        return m_ID;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake " + m_ID + " RPM", m_encoder.getVelocity());
    }

}
