package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class FastMidRung extends CommandBase {
    private final Climber m_climber;
    private boolean m_finished = false;
    private boolean m_pastHooks = false;
    double m_time = Double.POSITIVE_INFINITY;
    private final Timer m_timer = new Timer();
    
    public FastMidRung(Climber climber){
        m_climber = climber;        
    }
    

    @Override
    public void initialize(){
        m_climber.changeConstraints(new Constraints(90,30));
        m_climber.setDesiredPose(-10.0);
        m_finished = false;
        m_pastHooks = false;
        m_time = Double.POSITIVE_INFINITY;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Climbing from Floor", true);
        double currentTime = m_timer.get();

        /* if(m_climber.atSetpoint() && !m_slowDrop){
            m_climber.changeConstraints(new Constraints(30,15));
            m_climber.setDesiredPose(-10.0);
            m_slowDrop = true;
        } */

        if(m_climber.getLimit() && !m_pastHooks)
        {
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_climber.setDesiredPose(1.0);
            m_pastHooks = true;
        }
        else if(m_pastHooks && currentTime > m_time+0.250){
            m_climber.setDesiredPose(1.0);
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_climber.changeConstraints(new Constraints(50,25));
        m_climber.setDesiredPose(1.0);
        SmartDashboard.putBoolean("Climbing from Floor", false);
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

}