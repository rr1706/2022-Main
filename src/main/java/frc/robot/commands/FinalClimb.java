package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class FinalClimb extends CommandBase {
    private final Climber m_climber;
    private boolean m_finished = false;
    private boolean m_pastHooks = false;
    double m_time = Double.POSITIVE_INFINITY;
    private final Timer m_timer = new Timer();
    
    public FinalClimb(Climber climber){
        m_climber = climber;        
    }
    

    @Override
    public void initialize(){
        m_climber.changeConstraints(new Constraints(40,20));
        m_climber.setDesiredPose(35.0);
        m_finished = false;
        m_pastHooks = false;
        m_time = Double.POSITIVE_INFINITY;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){        
        SmartDashboard.putBoolean("Climbing To Bar", true);

        double currentTime = m_timer.get();
        if(m_climber.getLimit() && !m_pastHooks)
        {
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_climber.setDesiredPose(2.0);
            m_time = currentTime;
            m_pastHooks = true;
        }
        else if(m_pastHooks && currentTime > m_time+0.250){
            m_climber.setDesiredPose(6.0);
            m_finished = true;
        }
        else if(m_pastHooks){
            m_climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_climber.changeConstraints(new Constraints(50,25));
        m_climber.setDesiredPose(30.0);
        SmartDashboard.putBoolean("Climbing To Bar", false);
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

}
