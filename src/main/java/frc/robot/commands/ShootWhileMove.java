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

public class ShootWhileMove extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Drivetrain m_drive;
    private final ShooterHood m_hood;
    private final boolean m_updatePose;
    private final Timer m_timer = new Timer();
    private double m_shotTime = 0.0;

    private static Point2D[] m_shotTimes = 
        new Point2D.Double[]{
            //(ty-angle,time)
            new Point2D.Double(75,0.70),//
            new Point2D.Double(90,0.80),//
            new Point2D.Double(165,0.95),//
            new Point2D.Double(240,1.15)//
        };
    private static LinearInterpolationTable m_timeTable = new LinearInterpolationTable(m_shotTimes);

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
            new Point2D.Double(135,2670),//
            new Point2D.Double(150,2725),//
            new Point2D.Double(165,2875),//
            new Point2D.Double(180,3045
            ),//
            new Point2D.Double(195,3180),//
            new Point2D.Double(210,3315),//
            new Point2D.Double(240,3500),//
            new Point2D.Double(280,3850),
        };

    private static LinearInterpolationTable m_rpmTable = new LinearInterpolationTable(m_rpmPoints);

    

    public ShootWhileMove(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood, boolean updatePose){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        m_updatePose = updatePose;
        addRequirements(shooter, turret, hood);
    }

    @Override
    public void initialize(){
        m_turret.enable();
        m_turret.trackTarget(true);
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetHoodAdjust", 0.0);
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);
        m_shotTime=2.0;
    }

    @Override
    public void execute(){
        double currentTime = m_timer.get();

        SmartDashboard.putBoolean("Shooter Running", true);
        double goalVelX = -m_drive.getFieldRelativeSpeeds().vxMetersPerSecond;
        double goalVelY = -m_drive.getFieldRelativeSpeeds().vyMetersPerSecond;

        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d())*39.37;

        double fixedShotTime = m_timeTable.getOutput(dist);

        double virtualGoalX = GoalConstants.kGoalLocation.getX()+goalVelX*fixedShotTime;
        double virtualGoalY = GoalConstants.kGoalLocation.getY()+goalVelY*fixedShotTime;

        SmartDashboard.putNumber("Goal X", virtualGoalX);
        SmartDashboard.putNumber("Goal Y", virtualGoalY);

        Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);

        Translation2d toMovingGoal = movingGoalLocation.minus(m_drive.getPose().getTranslation());

        double newDist = toMovingGoal.getDistance(new Translation2d())*39.37;

            if(SmartDashboard.getBoolean("Adjust Shot?", false)){
                m_shooter.run(m_rpmTable.getOutput(newDist)+SmartDashboard.getNumber("SetShotAdjust", 0));
                m_hood.run(m_hoodTable.getOutput(newDist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
            } 
            else{
                m_shooter.run(m_rpmTable.getOutput(newDist));
                m_hood.run(m_hoodTable.getOutput(newDist));
            }

        m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation);

        if(currentTime > 0.250 && Limelight.valid()){// && !robotMovingFast(m_drive.getChassisSpeed())){
            double dL = Limelight.getDistance()*0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement()-Math.PI;
            double tL = -1.0*Limelight.tx();
    
            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);
    
            if(m_updatePose){
                m_drive.setPose(pose);
            }
    
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
      m_turret.trackTarget(false);
      m_turret.disable();
      m_turret.stop();
      m_shooter.stop();
      m_hood.stop();
      m_timer.stop();
    }
    
    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal){
        double tG = tR+tT+tL;
        double rX = goal.getX()-dL*Math.cos(tG);
        double rY = goal.getY()-dL*Math.sin(tG);
    
        return new Pose2d(rX,rY, new Rotation2d(-tR));
    }
    
    private boolean robotMovingFast(ChassisSpeeds input){
        double speed = Math.sqrt(Math.pow(input.vxMetersPerSecond,2)+Math.pow(input.vyMetersPerSecond, 2));
        double angleSpeed = Math.abs(input.omegaRadiansPerSecond);
    
        if(speed > 1.000 || angleSpeed > 0.050){
            return true;
        }
        else{
            return false;
        }
    
    }

}
