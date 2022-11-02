package frc.robot.commands.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class ClimbToHighBar extends CommandBase {
    private final Climber m_climber;
    private final Drivetrain m_drive;
    private boolean m_finished = false;
    private boolean m_pastHooks = false;
    private boolean m_setDown = false;
    private boolean m_slowStartDone = false;
    private double m_omega;
    private double m_alpha;
    private double m_lastTheta;
    private double m_lastOmega;
    double m_time = Double.POSITIVE_INFINITY;
    private final Timer m_timer = new Timer();

    public ClimbToHighBar(Climber climber, Drivetrain drive) {
        m_climber = climber;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_climber.changeConstraints(new Constraints(90, 15));
        m_climber.setDesiredPose(-15.0);
        m_finished = false;
        m_pastHooks = false;
        m_setDown = false;
        m_time = Double.POSITIVE_INFINITY;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Climbing from Floor", true);
        double currentTime = m_timer.get();

        m_omega = m_drive.getGyroXVel();
        m_alpha = (m_omega-m_lastOmega)/GlobalConstants.kLoopTime;
        m_lastOmega = m_omega;

        SmartDashboard.putNumber("omega", m_omega);
        SmartDashboard.putNumber("alpha", m_alpha);

        if(m_climber.getPose()<60.0 && !m_slowStartDone){
            m_climber.changeConstraints(new Constraints(90, 25));
            m_slowStartDone = true;
        }

        if (m_climber.getLimit() && !m_pastHooks) {
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_climber.setDesiredPose(-3.0);
            m_time = currentTime;
            m_pastHooks = true;
        } else if (m_pastHooks && currentTime > m_time + 2.0) {
            m_climber.setDesiredPose(8.0);
            m_setDown = true;
            //m_finished = true;
        }
        if(m_setDown){
            if(Math.abs(m_omega) <= 4.00 && m_alpha >= 2.00){
                m_finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.changeConstraints(ClimberConstants.kDefaultConstraints);
        m_climber.setDesiredPose(8.0);
        SmartDashboard.putBoolean("Climbing from Floor", false);
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
