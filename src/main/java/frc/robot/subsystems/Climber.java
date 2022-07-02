package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {

    private double m_pose = 1.0;
    private final CANSparkMax m_motor1 = new CANSparkMax(ClimberConstants.kMotorID[0], MotorType.kBrushless);
    private final CANSparkMax m_motor2 = new CANSparkMax(ClimberConstants.kMotorID[1], MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    private final DoubleSolenoid m_valve = new DoubleSolenoid(GlobalConstants.PCHID, PneumaticsModuleType.REVPH,
            ClimberConstants.kValvePorts[0], ClimberConstants.kValvePorts[1]);
    private final ProfiledPIDController m_PID = new ProfiledPIDController(ClimberConstants.kPID[0],
            ClimberConstants.kPID[1], ClimberConstants.kPID[2], ClimberConstants.kDefaultConstraints);
    private final DigitalInput m_limit = new DigitalInput(0);

    public Climber() {
        m_motor1.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_motor2.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor1.setInverted(true);
        m_motor2.follow(m_motor1, true);

        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);

        m_motor1.burnFlash();
        m_motor2.burnFlash();

    }

    public void changeConstraints(Constraints newConstraints) {
        m_PID.setConstraints(newConstraints);
    }

    public void extend() {
        m_valve.set(Value.kForward);
    }

    public void retract() {
        m_valve.set(Value.kReverse);
    }

    // Change length of arm
    public void setDesiredPose(double pose) {
        m_PID.reset(getPose());
        m_pose = pose;
    }

    public void setPower(double power) {
        m_motor1.set(power);
    }

    public void setPoseRef(double pose) {
        m_encoder.setPosition(pose);
    }

    public void run() {
        if (m_pose < ClimberConstants.kMinPose) {
            m_pose = ClimberConstants.kMinPose;
        } else if (m_pose > ClimberConstants.kMaxPose) {
            m_pose = ClimberConstants.kMaxPose;
        }
        m_motor1.set(m_PID.calculate(m_encoder.getPosition(), m_pose));
    }

    @Override
    public void periodic() {
        double pose = m_encoder.getPosition();

        SmartDashboard.putNumber("Climber Pose", pose);
        SmartDashboard.putNumber("Climber Current", getCurrent());
        SmartDashboard.putBoolean("Climber Limit", getLimit());

    }

    public double getPose() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }

    public double getCurrent() {
        return m_motor1.getOutputCurrent() + m_motor2.getOutputCurrent();
    }

    public boolean atSetpoint() {
        return Math.abs(m_pose - getPose()) < 1.0;
    }

    public boolean getLimit() {
        return m_limit.get();
    }
}
