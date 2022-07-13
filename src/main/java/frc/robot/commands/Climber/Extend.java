package frc.robot.commands.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class Extend extends CommandBase {
    private final Climber m_climber;
    private boolean m_hasExtended = false;

    public Extend(Climber climber) {
        m_climber = climber;
    }

    @Override
    public void initialize() {
        m_climber.changeConstraints(new Constraints(100, 250));
        m_climber.setDesiredPose(69.5);
        m_hasExtended = false;
    }

    @Override
    public void execute() {
        double pose = m_climber.getPose();
        if (pose >= 15 && !m_hasExtended) {
            m_climber.retract();
        }
        if (m_climber.atSetpoint() && !m_hasExtended) {
            m_climber.extend();
            m_hasExtended = true;

        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.changeConstraints(ClimberConstants.kDefaultConstraints);

    }

    @Override
    public boolean isFinished() {
        return m_hasExtended;
    }
}
