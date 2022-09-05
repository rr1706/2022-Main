package frc.robot.commands.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class ClimbFromFloor extends CommandBase {
    private final Climber m_climber;
    private final Drivetrain m_drive;
    private boolean m_finished = false;
    private boolean m_setDown = false;
    private boolean m_pastHooks = false;
    private double m_omega;
    private double m_alpha;
    private double m_lastOmega;
    double m_time = Double.POSITIVE_INFINITY;
    private final Timer m_timer = new Timer();

    public ClimbFromFloor(Climber climber, Drivetrain drive) {
        m_climber = climber;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_climber.changeConstraints(new Constraints(90, 30));
        m_climber.setDesiredPose(-15.0);
        m_finished = false;
        m_pastHooks = false;
        m_setDown = false;
        m_time = Double.POSITIVE_INFINITY;
        m_timer.reset();
        m_timer.start();

        m_lastOmega = m_drive.getGyroXVel();
        m_alpha = 0.0;
    }

    @Override
    public void execute() {
        double currentTime = m_timer.get();
        m_omega = m_drive.getGyroXVel();
        m_alpha = (m_omega-m_lastOmega)/GlobalConstants.kLoopTime;



        m_lastOmega = m_omega;

        if (m_climber.getLimit() && !m_pastHooks) {
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_climber.setDesiredPose(-3.0);
            m_time = currentTime;
            m_pastHooks = true;
        } else if (m_pastHooks && currentTime > m_time + 0.25) {
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
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
