package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class FeedShooter extends CommandBase{
    private final Turret m_turret;
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Drivetrain m_drive;
    private final Timer m_timer = new Timer();
    private boolean m_End = false;

    public FeedShooter(Turret turret, Elevator top, Elevator bottom, Drivetrain
     drive){
        m_turret = turret;
        m_top = top;
        m_bottom = bottom;
        m_drive = drive;
        addRequirements(top,bottom);
    }

    @Override
    public void initialize(){
        m_top.run();
        //m_bottom.run();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        boolean isRobotMoving = Math.abs(m_drive.getChassisSpeed().vxMetersPerSecond)>0.25 || Math.abs(m_drive.getChassisSpeed().vyMetersPerSecond)>0.25 || Math.abs(m_drive.getChassisSpeed().omegaRadiansPerSecond)>0.25;
        boolean canShoot = !isRobotMoving;
            if(canShoot){
                m_top.run();
                if(m_timer.get()>0.100){
                    m_bottom.run();
                }
            }
            else{
                m_top.stop();
                m_bottom.stop();
            }
            
            
            SmartDashboard.putBoolean("Shooting", true);
        
        
    }
    @Override
    public void end(boolean interrupted){
        m_timer.stop();
        m_top.stop();
        m_End = false;
        SmartDashboard.putBoolean("Shooting", false);
    }

    public void stop(){
        m_End = true;
    }
    @Override
    public boolean isFinished(){
        return m_End;
    }
}
