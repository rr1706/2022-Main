package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
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

    private static Point2D[] m_hoodPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,0.0),
            new Point2D.Double(50,0.0),
            new Point2D.Double(75,10.0),//
            new Point2D.Double(90,15.5),//
            new Point2D.Double(105,19.0),//
            new Point2D.Double(120,23.0),//
            new Point2D.Double(135,26.0),//
            new Point2D.Double(150,28.0),//
            new Point2D.Double(165,29.5),//
            new Point2D.Double(180,33.0),//
            new Point2D.Double(195,36.0),//
            new Point2D.Double(210,38.0),//
            new Point2D.Double(240,38.0)//
        };
    private static LinearInterpolationTable m_hoodTable = new LinearInterpolationTable(m_hoodPoints);

    private static Point2D[] m_rpmPoints = 
        new Point2D.Double[]{
            //(ty-angle,distance)
            new Point2D.Double(35,2350),
            new Point2D.Double(50,2350),
            new Point2D.Double(75,2340),//
            new Point2D.Double(90,2450),//
            new Point2D.Double(105,2480),//
            new Point2D.Double(120,2570),//
            new Point2D.Double(135,2690),//
            new Point2D.Double(150,2770),//
            new Point2D.Double(165,2875),//
            new Point2D.Double(180,3045),//
            new Point2D.Double(195,3180),//
            new Point2D.Double(210,3315),//
            new Point2D.Double(240,3500),//
            new Point2D.Double(280,3850),
        };

    private static LinearInterpolationTable m_rpmTable = new LinearInterpolationTable(m_rpmPoints);

    

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
        m_turret.aimAtGoal(m_drive.getPose(), GoalConstants.kHangerLocation);
        m_shooter.run(1500);
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
