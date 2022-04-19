package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class IndexElevator extends CommandBase{
    
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final XboxController m_operator;

    public IndexElevator(Elevator top, Elevator bottom, XboxController operator){
        m_top = top;
        m_bottom = bottom;
        m_operator = operator;
        addRequirements(bottom);        
    }
    public IndexElevator(Elevator top, Elevator bottom){
        m_top = top;
        m_bottom = bottom;
        m_operator = new XboxController(3);
        addRequirements(bottom);        
    }
    @Override
    public void execute(){
        if(m_top.getSensor()&&m_bottom.getSensor()){
            m_bottom.stop();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", true);
            m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
            m_operator.setRumble(RumbleType.kRightRumble, 1.0);
        }else if(!m_bottom.getSensor()&& !m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", false);
            m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
            m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        }else if(m_top.getSensor()&&!m_bottom.getSensor()){
            m_bottom.run();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", false);
            m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
            m_operator.setRumble(RumbleType.kRightRumble, 1.0);
        }else if(m_bottom.getSensor()&&!m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", true);
            m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
            m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
        m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        m_bottom.stop();
        m_top.stop();
    }
}
