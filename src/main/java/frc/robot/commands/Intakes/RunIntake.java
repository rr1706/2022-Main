package frc.robot.commands.Intakes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private final Intake m_intake;

    public RunIntake(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.extend();
    }

    @Override
    public void execute() {
        //SmartDashboard.putBoolean(m_intake.getID() + " Running", true);
        m_intake.run(IntakeConstants.kSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean(m_intake.getID() + " Running", false);
        m_intake.retract();
    }
}
