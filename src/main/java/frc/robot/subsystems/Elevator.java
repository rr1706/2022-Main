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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_PID;
    private final String m_ID;
    private final DigitalInput m_sensor;

    public Elevator(int motorCANID, int sensorPort, String ID) {
        m_ID = ID;
        m_sensor = new DigitalInput(sensorPort);
        m_motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_PID = m_motor.getPIDController();

        m_motor.setSmartCurrentLimit(CurrentLimit.kElevator);
        m_motor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PID.setOutputRange(-1.0, 1.0);
        m_PID.setP(ElevatorConstants.kP);
        m_PID.setFF(ElevatorConstants.kFF);
        m_motor.burnFlash();
    }

    public void run() {
        m_PID.setReference(ElevatorConstants.kSpeed, ControlType.kVelocity);
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
        SmartDashboard.putNumber(m_ID + " Elevator RPM", m_encoder.getVelocity());
    }
}
