package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class SmartIntake extends CommandBase {
    private final Intake m_rightIntake;
    private final Intake m_leftIntake;
    private final Drivetrain m_drive;
    private final Timer m_timer = new Timer();
    private boolean m_rightRunning = false;
    private boolean m_leftRunning = false;
    private double m_leftStopTime = 0.0;
    private double m_rightStopTime = 0.0;
    private double yVelAve = 0.0;

    public SmartIntake(Intake right, Intake left, Drivetrain drive) {
        m_leftIntake = left;
        m_rightIntake = right;
        m_drive = drive;
        addRequirements(left, right);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        double yVel = m_drive.getChassisSpeed().vyMetersPerSecond;

        if (yVel > 0.05) {
            m_leftIntake.extend();
            m_leftIntake.run(IntakeConstants.kSpeed);
            m_leftRunning = true;
        } else if (yVel < -0.05) {
            m_rightIntake.extend();
            m_rightIntake.run(IntakeConstants.kSpeed);
            m_rightRunning = true;
        }

    }

    @Override
    public void execute() {
        final double currentTime = m_timer.get();
        double yVel = m_drive.getChassisSpeed().vyMetersPerSecond;
        yVelAve = (3 * yVelAve + yVel) / 4.0;

        if (yVelAve > 0.5) {
            yVelAve = 0.5;
        } else if (yVelAve < -0.5) {
            yVelAve = -0.5;
        }

        if (yVelAve < -0.20) {
            if (m_leftRunning) {
                m_leftIntake.retract();
                m_leftRunning = false;
                m_leftStopTime = currentTime;
            }

            m_rightIntake.extend();
            m_rightIntake.run(IntakeConstants.kSpeed);
        } else if (yVelAve > 0.20) {
            if (m_rightRunning) {
                m_rightIntake.retract();
                m_rightRunning = false;
                m_rightStopTime = currentTime;
            }
            m_leftRunning = true;
            m_leftIntake.extend();
            m_leftIntake.run(IntakeConstants.kSpeed);
        }

        if (!m_leftRunning && currentTime > m_leftStopTime + 0.80) {
            m_leftIntake.stop();
        }

        if (!m_rightRunning && currentTime > m_rightStopTime + 0.80) {
            m_rightIntake.stop();
        }

        SmartDashboard.putBoolean(m_leftIntake.getID() + " Running", m_leftRunning);
        SmartDashboard.putBoolean(m_rightIntake.getID() + " Running", m_rightRunning);

    }

    @Override
    public void end(boolean interrupted) {
        m_rightRunning = false;
        m_leftRunning = false;
        SmartDashboard.putBoolean(m_leftIntake.getID() + " Running", false);
        SmartDashboard.putBoolean(m_rightIntake.getID() + " Running", false);
        m_leftStopTime = 0.0;
        m_rightStopTime = 0.0;
        m_leftIntake.retract();
        m_rightIntake.retract();
        yVelAve = 0.0;
        m_timer.stop();
    }
}
