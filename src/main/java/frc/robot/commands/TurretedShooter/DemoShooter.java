package frc.robot.commands.TurretedShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class DemoShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final ShooterHood m_hood;

    private static LinearInterpolationTable m_hoodTable = ShooterConstants.kHoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

    public DemoShooter(Shooter shooter, Turret turret, ShooterHood hood){
        m_shooter = shooter;
        m_turret = turret;
        m_hood = hood;   

        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize(){
        m_turret.setToStartPosition();
        Limelight.disable();
        m_shooter.run(1000);
        m_hood.run(10.0);

        SmartDashboard.putNumber("DemoDistance", 100);
    }

    @Override
    public void execute(){
        double rpm = m_rpmTable.getOutput(SmartDashboard.getNumber("DemoDistance", 100))-750.0;
        double hood = m_hoodTable.getOutput(SmartDashboard.getNumber("DemoDistance", 100))+10.0;

        m_shooter.run(rpm);
        m_hood.run(hood);
        m_turret.stop();
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
        m_hood.stop();
        m_turret.stop();
    }

    
}
