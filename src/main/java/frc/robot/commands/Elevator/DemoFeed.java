package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class DemoFeed extends CommandBase {
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final XboxController m_driver;

    public DemoFeed(Elevator top, Elevator bottom, XboxController driver){
        m_top = top;
        m_bottom = bottom;
        m_driver = driver;
        addRequirements(bottom);
    }

    @Override
    public void initialize(){
        m_top.stop();
        m_bottom.stop();
    }

    @Override
    public void execute(){
        if(m_driver.getLeftTriggerAxis() > 0.5){
            m_top.run();
            m_bottom.run();
        }
        else{
            m_top.stop();
            m_bottom.stop();
        }
    }

    
}
