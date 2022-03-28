package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimb extends CommandBase {
    private final Climber m_climber;
    private boolean m_LimitHit = false;
    private final Timer m_timer = new Timer();
    private boolean m_finished = false;
    private double m_hitTime = Double.POSITIVE_INFINITY;

    public ZeroClimb(Climber climber){
        m_climber = climber;
        addRequirements(m_climber);
    }
    @Override
    public void initialize(){
        m_climber.setPower(-0.10);
        m_finished = false;
        m_timer.reset();
        m_timer.start();
        m_LimitHit = false;
        m_hitTime = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Zeroing Climber", true);
        double time = m_timer.get();
        if(m_climber.getLimit() && time>0.040 && !m_LimitHit){
            m_climber.stop();
            m_climber.setPoseRef(0.0);
            m_LimitHit = true;
            m_hitTime = time;
        }
        else if(m_LimitHit && time > m_hitTime+0.050){
            m_climber.setDesiredPose(2.0);
            m_finished = true;
        }
        else if(m_LimitHit){
            m_climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Zeroing Climber", false);
        m_climber.setDesiredPose(2.0);      
        m_climber.stop();
        m_timer.stop();

    }
    @Override
    public boolean isFinished(){
        return m_finished;
    }
}
