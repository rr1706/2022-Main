package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private final Intake m_intake;
    private final Timer m_timer = new Timer();
    private double m_ballTime = 0.00;
    private boolean m_ballDetected = false;
    public RunIntake(Intake intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_intake.extend();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean(m_intake.getID() + " Running", true);
        final double currentTime = m_timer.get();
        if(m_intake.getCurrent()>12.5 && currentTime>0.250 && !m_ballDetected)
        {
            m_ballTime = currentTime;
            m_ballDetected = true;
            m_intake.run(11000);
        }
        else if(!m_ballDetected){
            m_intake.run(11000);
        }
        else if(m_ballDetected && Math.abs(m_ballTime-currentTime)>0.50){
            m_ballDetected = false;
            m_ballTime = 0.0;
            m_timer.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(m_intake.getID() + " Running", false);
        m_intake.retract();
        m_timer.stop();
        m_ballDetected = false;
        m_ballTime = 0.0;
    }
}
