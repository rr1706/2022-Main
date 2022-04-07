package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import frc.robot.Utilities.LinearInterpolationTable;
import frc.robot.subsystems.ColorSensor;
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
    private final ColorSensor m_color;
    private final Timer m_timer = new Timer();

    private static Point2D[] m_shotTimes = 
        new Point2D.Double[]{
            //(ty-angle,time)
            new Point2D.Double(75,0.85),//
            new Point2D.Double(90,0.85),//
            new Point2D.Double(165,0.85),//
            new Point2D.Double(240,1.05)//
        };
    private static LinearInterpolationTable m_timeTable = new LinearInterpolationTable(m_shotTimes);

    private static LinearInterpolationTable m_hoodTable = ShooterConstants.khoodTable;
    private static LinearInterpolationTable m_rpmTable = ShooterConstants.krpmTable;
    

    public ShootWhileMove(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood, boolean updatePose, ColorSensor color){
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_hood = hood;
        m_updatePose = updatePose;
        m_color = color;
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
    }

    @Override
    public void execute(){

        boolean wrongColor = !m_color.isAllianceBall();

        double currentTime = m_timer.get();        
        SmartDashboard.putBoolean("Shooter Running", true);

        FieldRelativeSpeed robotVel = m_drive.getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d())*39.37;

        double fixedShotTime = m_timeTable.getOutput(dist);

        double virtualGoalX = GoalConstants.kGoalLocation.getX()-fixedShotTime*(robotVel.vx+robotAccel.ax*ShooterConstants.kAccelCompFactor);
        double virtualGoalY = GoalConstants.kGoalLocation.getY()-fixedShotTime*(robotVel.vy+robotAccel.ay*ShooterConstants.kAccelCompFactor);

        SmartDashboard.putNumber("Goal X", virtualGoalX);
        SmartDashboard.putNumber("Goal Y", virtualGoalY);

        Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);

        Translation2d toMovingGoal = movingGoalLocation.minus(m_drive.getPose().getTranslation());

        double newDist = toMovingGoal.getDistance(new Translation2d())*39.37;

            if(SmartDashboard.getBoolean("Adjust Shot?", false)){
                m_shooter.run(ShooterConstants.krpmTable.getOutput(newDist)+SmartDashboard.getNumber("SetShotAdjust", 0));
                m_hood.run(m_hoodTable.getOutput(newDist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
            }
            else if(wrongColor){
                m_shooter.run(1500);
                m_hood.run(1.0);
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

}
