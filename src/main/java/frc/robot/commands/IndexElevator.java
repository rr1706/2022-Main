package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rumble;

public class IndexElevator extends CommandBase{
    
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Rumble m_driverRumble;
    private final Rumble m_operatorRumble;

    private int m_temp = 0;

    public IndexElevator(Elevator top, Elevator bottom, XboxController driverController, XboxController operatorController) {
        m_top = top;
        m_bottom = bottom;
        m_driverRumble = new Rumble(driverController);
        m_operatorRumble = new Rumble(operatorController);
        addRequirements(bottom);        
    }
    
    @Override
    public void execute(){
        if(m_top.getSensor()&&m_bottom.getSensor()){
            m_bottom.stop();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", true);
            if (m_temp != 3) {
                m_temp = 3;
                m_driverRumble.setRumble(1, 0.5, 0.1);
            }
        }else if(!m_bottom.getSensor()&& !m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", false);
            m_temp = 0;
        }else if(m_top.getSensor()&&!m_bottom.getSensor()){
            m_bottom.run();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", false);
            if (m_temp != 2) {
                m_temp = 2;
                m_driverRumble.setRumble(2, 0.5, 0.1);
            }
        }else if(m_bottom.getSensor()&&!m_top.getSensor()){
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", true);
            if (m_temp != 3) {
                m_temp = 1;
                m_driverRumble.setRumble(2, 0.5, 0.1);            
            }
        }
    }
}
