package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class UnjamIntake extends CommandBase {
    private final Intake m_intake;
    public UnjamIntake(Intake intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_intake.extend();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean(m_intake.getID() + " Running", true);
        m_intake.run(-3000);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(m_intake.getID() + " Running", false);
        m_intake.stop();
        m_intake.retract();
    }
}
