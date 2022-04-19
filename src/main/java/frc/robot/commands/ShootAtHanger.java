package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;
import java.awt.geom.Point2D;

public class ShootAtHanger extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;

    private static LinearInterpolationTable m_hoodTable = ShooterConstants.khoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.krpmTable;

    

    public ShootAtHanger(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize(){
        m_turret.enable();
        m_turret.trackTarget(false);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Shooter Running", true);
        m_turret.aimAtGoal(m_drive.getPose(), GoalConstants.kHangerLocation, false);
        m_shooter.run(1200);
        m_hood.run(38.0);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
      m_turret.trackTarget(false);
      m_turret.disable();
      m_turret.stop();
      m_shooter.stop();
      m_hood.stop();
    }

}
