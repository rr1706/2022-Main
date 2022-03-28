package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class ZeroHood extends CommandBase {
    private ShooterHood m_hood;
    private Timer m_timer = new Timer();
    private boolean m_finished = false;

    public ZeroHood(ShooterHood hood){
        m_hood = hood;
        addRequirements(m_hood);
    }
    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_finished = false;
        m_hood.setHood(-0.10);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Zeroing Hood", true);
        double time = m_timer.get();
        double current = m_hood.getTotalCurrent();
        if(current>19.5 && time >0.040){
            m_hood.stop();
            m_hood.setHoodZero();
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Zeroing Hood", false);
        m_timer.stop();
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

}
