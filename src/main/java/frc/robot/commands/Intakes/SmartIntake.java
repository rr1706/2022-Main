package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class SmartIntake extends CommandBase {
    private final Intake m_rightIntake;
    private final Intake m_leftIntake;
    private final Drivetrain m_drive;
    private final XboxController m_controller;
    private final Timer m_timer = new Timer();
    private boolean m_rightRunning = false;
    private boolean m_leftRunning = false;
    private double m_leftStopTime = 0.0;
    private double m_rightStopTime = 0.0;
    private boolean m_finished = false;

    public SmartIntake(Intake right, Intake left, Drivetrain drive, XboxController controller) {
        m_leftIntake = left;
        m_rightIntake = right;
        m_drive = drive;
        m_controller = controller;
        addRequirements(left, right);
    }

    @Override
    public void initialize() {
        m_finished = false;
        m_timer.reset();
        m_timer.start();
        final double yVel = m_drive.getChassisSpeed().vyMetersPerSecond;

        if (yVel >= 0.0) {
            m_leftIntake.extend();
            m_leftIntake.run(IntakeConstants.kSpeed);
            m_leftRunning = true;
        } else if (yVel < 0.0) {
            m_rightIntake.extend();
            m_rightIntake.run(IntakeConstants.kSpeed);
            m_rightRunning = true;
        }

    }

    @Override
    public void execute() {
        final double currentTime = m_timer.get();
        final double yVel = m_drive.getChassisSpeed().vyMetersPerSecond;
        if (m_controller.getXButtonPressed() || m_leftRunning && yVel < -1.00 || m_rightRunning && yVel > 1.00) {
            if (m_leftRunning) {
                m_leftIntake.retract();
                m_leftRunning = false;
                m_leftStopTime = currentTime;
                m_rightRunning = true;
                m_rightIntake.extend();
                m_rightIntake.run(IntakeConstants.kSpeed);
            }
            else if (m_rightRunning) {
                m_rightIntake.retract();
                m_rightRunning = false;
                m_rightStopTime = currentTime;
                m_leftRunning = true;
                m_leftIntake.extend();
                m_leftIntake.run(IntakeConstants.kSpeed);
            }
        }

        if (!m_leftRunning && currentTime > m_leftStopTime + 0.80) {
            m_leftIntake.stop();
        }

        if (!m_rightRunning && currentTime > m_rightStopTime + 0.80) {
            m_rightIntake.stop();
        }



    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_rightRunning = false;
        m_leftRunning = false;
        m_leftStopTime = 0.0;
        m_rightStopTime = 0.0;
        m_leftIntake.retract();
        m_rightIntake.retract();
        m_timer.stop();
    }
}
