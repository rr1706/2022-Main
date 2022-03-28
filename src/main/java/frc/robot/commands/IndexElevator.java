package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class IndexElevator extends CommandBase{
    
    private final Elevator m_top;
    private final Elevator m_bottom;

    public IndexElevator(Elevator top, Elevator bottom){
        m_top = top;
        m_bottom = bottom;
        addRequirements(bottom);        
    }
    @Override
    public void execute(){
        if(m_top.getSensor()&&m_bottom.getSensor()){
            m_bottom.stop();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", true);
        }else if(!m_bottom.getSensor()&& !m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", false);
        }else if(m_top.getSensor()&&!m_bottom.getSensor()){
            m_bottom.run();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", false);
        }else if(m_bottom.getSensor()&&!m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", true);
        }
    }
}
