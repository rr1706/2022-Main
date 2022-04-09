package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class MoveFeed extends CommandBase{
    private final Turret m_turret;
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Drivetrain m_drive;
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final Rumble m_driverRumble;
    private final Timer m_timer = new Timer();
    private boolean m_End = false;
    private double m_lastFeedTime = 0.0;

    private int m_temp = 0;

    public MoveFeed(Turret turret, Elevator top, Elevator bottom, Drivetrain
     drive, Shooter shooter, ShooterHood hood, XboxController controller){
        m_turret = turret;
        m_top = top;
        m_bottom = bottom;
        m_drive = drive;
        m_shooter = shooter;
        m_hood = hood;
        m_driverRumble = new Rumble(controller);
        addRequirements(top,bottom);
    }

    @Override
    public void initialize(){
        m_top.stop();
        m_bottom.stop();
        m_End = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        boolean isOmegaLow = Math.abs(m_drive.getFieldRelativeSpeed().omega) < Math.PI/3.0;
        boolean isAlphaLow = Math.abs(m_drive.getFieldRelativeAccel().alpha) <= 1.00;
        boolean isJerkLow = Math.abs(m_drive.getFieldRelativeJerk().jx) <= 1.00 
            && Math.abs(m_drive.getFieldRelativeJerk().jy) <= 1.00
            && Math.abs(m_drive.getFieldRelativeJerk().jz) <= 1.00;
        boolean isTurretReady = m_turret.atDesiredAngle();
        boolean isShooterReady = m_shooter.atSetpoint();
        boolean isHoodReady = m_hood.atSetpoint();
        boolean canShoot = isOmegaLow && isAlphaLow && isJerkLow && isTurretReady && isShooterReady && isHoodReady;
            if(canShoot){
                m_top.run();
                if(m_timer.get()>0.100){
                    m_bottom.run();
                }
            }
            else{
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
            
            
            SmartDashboard.putBoolean("Shooting", true);
        
        
    }
    @Override
    public void end(boolean interrupted){
        m_timer.stop();
        m_top.stop();
        m_bottom.stop();
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
