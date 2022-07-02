package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_PID;
    private final String m_ID;
    private final DigitalInput m_sensor;
    private double m_RPM = 10000;

    public Elevator(int motorCANID, int sensorPort, String ID, double rpm) {
        m_RPM = rpm;
        m_ID = ID;
        m_sensor = new DigitalInput(sensorPort);
        m_motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_PID = m_motor.getPIDController();

        m_motor.setSmartCurrentLimit(CurrentLimit.kElevator);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PID.setOutputRange(-1.0, 1.0);
        m_PID.setP(0.0001);
        m_PID.setI(0.0);
        m_PID.setD(0.0);
        m_PID.setFF(0.000097);
        m_motor.burnFlash();

        // SmartDashboard.putNumber("Set "+m_ID+" Elevator RPM", m_RPM);

    }

    public void run() {
        m_PID.setReference(m_RPM, ControlType.kVelocity);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public boolean getSensor() {
        return !m_sensor.get();
    }

    @Override
    public void periodic() {
        // m_RPM = SmartDashboard.getNumber("Set "+m_ID+" Elevator RPM", 10000);
        // SmartDashboard.putNumber(m_ID+" Elevator Current",
        // m_motor.getOutputCurrent());
        // SmartDashboard.putNumber(m_ID+" Elevator Motor Temp",
        // m_motor.getMotorTemperature());
        SmartDashboard.putNumber(m_ID + " Elevator RPM", m_encoder.getVelocity());
        // SmartDashboard.putBoolean(m_ID+" Indexed Ball", getSensor());
    }
}
