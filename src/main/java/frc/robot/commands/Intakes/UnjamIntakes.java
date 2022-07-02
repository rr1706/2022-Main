package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class UnjamIntakes extends CommandBase {
    private final Intake m_leftIntake;
    private final Intake m_rightIntake;

    public UnjamIntakes(Intake leftIntake, Intake rightIntake) {
        m_leftIntake = leftIntake;
        m_rightIntake = rightIntake;
        addRequirements(m_leftIntake, m_rightIntake);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean(m_leftIntake.getID() + " Running", true);
        SmartDashboard.putBoolean(m_rightIntake.getID() + " Running", true);
        m_leftIntake.run(IntakeConstants.kRevSpeed);
        m_rightIntake.run(IntakeConstants.kRevSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(m_leftIntake.getID() + " Running", false);
        SmartDashboard.putBoolean(m_rightIntake.getID() + " Running", false);
        m_leftIntake.stop();
        m_rightIntake.stop();
    }
}
