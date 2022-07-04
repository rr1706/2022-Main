package frc.robot.Utilities.PathPlannerUtils;

import java.lang.String;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoFromPathPlanner extends SequentialCommandGroup {
  private PathPlannerTrajectory m_trajectory;

  public AutoFromPathPlanner(Drivetrain drive, String pathName, double maxSpeed, boolean endStationary) {
    m_trajectory = PathPlanner.loadPath(pathName, maxSpeed, AutoConstants.kMaxAcceleration);

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    AutoSwerveController swerveControllerCommand = new AutoSwerveController(m_trajectory, drive::getAutoPose,
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, drive::setModuleStates, drive);

    addRequirements(drive);

    if(endStationary){
    // Run path following command, then stop at the end.
      addCommands(
         new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
          swerveControllerCommand, 
          new InstantCommand(() -> drive.stop()));
    }
    else{
          // Run path following command, robot will continue moving at same swerve module commanded state at the end 
          // (NEXT COMMAND IN AUTO MUST BE MOVEMENT OR ELSE ROBOT MAY CONTINUE MOVING UNCONTROLLABLY)
          addCommands(new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
          swerveControllerCommand);
    }


  }

  public Pose2d getInitialPose() {
    return new Pose2d(m_trajectory.getInitialState().poseMeters.getX(),
        m_trajectory.getInitialState().poseMeters.getY(),
        m_trajectory.getInitialState().holonomicRotation.times(-1.0));
  }
}
