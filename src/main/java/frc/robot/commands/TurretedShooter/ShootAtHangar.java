package frc.robot.commands.TurretedShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class ShootAtHangar extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;

    public ShootAtHangar(Shooter shooter, Turret turret, Drivetrain drive, ShooterHood hood) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize() {
        m_turret.trackTarget(false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Shooter Running", true);
        m_turret.aimAtGoal(m_drive.getPose(), GoalConstants.kHangerLocation, false);
        m_shooter.run(ShooterConstants.kHangarRPM);
        m_hood.run(HoodConstants.kMaxAngle);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        m_turret.trackTarget(false);
        m_turret.stop();
        m_shooter.stop();
        m_hood.stop();
    }

}
