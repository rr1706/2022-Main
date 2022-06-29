package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities.MathUtils;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve.Drivetrain;

public class FOIntake extends CommandBase {
    private final Intake m_rightIntake;
    private final Intake m_leftIntake;
    private Intake m_intake;
    private final Drivetrain m_drive;
    private final Timer m_timer = new Timer();
    private boolean m_side = false;
    private boolean m_forceRO = false;

    public FOIntake(Intake right, Intake left, boolean side, Drivetrain drive){
        m_leftIntake = left;
        m_rightIntake = right;
        m_drive = drive;
        m_side = side;
        addRequirements(left,right);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        double angle = MathUtils.toUnitCircAngle(m_drive.getGyro().getRadians());
        double xVel = m_drive.getFieldRelativeSpeed().vx;
        if((angle >= 2*Math.PI/6 && angle < 4*Math.PI/6)){
            if(xVel >= 0.0){
                m_intake = m_rightIntake;
            }
            else{
                m_intake = m_leftIntake;
            }
        }
        else if((angle >=8*Math.PI/6 && angle < 10*Math.PI/6)){
            if(xVel < 0.0){
                m_intake = m_rightIntake;
            }
            else{
                m_intake = m_leftIntake;
            }
        }
        else if(angle>=((3*Math.PI)/2)||angle<=Math.PI/2 ){
            if(m_side){
                m_intake = m_rightIntake;
            }else{
                m_intake = m_leftIntake;
            }
        }else{
            if(!m_side){
                m_intake = m_rightIntake;
            }else{
                m_intake = m_leftIntake;
            }
        }
        if(m_forceRO){
            if(m_side){
                m_intake = m_rightIntake;
            }
            else{
                m_intake = m_leftIntake;
            }
        }
        m_intake.extend();
        
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean(m_intake.getID() + " Running", true);
        final double currentTime = m_timer.get();
/*         if(m_intake.getCurrent()>12.5 && currentTime>0.250 && !m_ballDetected)
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
        } */
        m_intake.run(11000);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean(m_intake.getID() + " Running", false);
        m_intake.retract();
        m_timer.stop();
    }

    public void toggleRO(){
        if(m_forceRO){
            m_forceRO = false;
        }
        else{
            m_forceRO = true;
        }
    }
}
