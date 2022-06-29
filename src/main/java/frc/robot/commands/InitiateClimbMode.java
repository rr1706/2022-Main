package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utilities.MathUtils;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class InitiateClimbMode extends CommandBase {
    private final Climber m_climber;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Turret m_turret;
    private final Intake m_leftIntake;
    private final Intake m_rightIntake;
    private final Elevator m_highElevator;
    private final Elevator m_lowElevator;
    private final Drivetrain m_drive; 
    private final XboxController m_controller;   
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(1.45);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(1.45);
   private boolean m_climbModeReady = false;

    public InitiateClimbMode(Shooter shooter, ShooterHood hood, Turret turret, Intake left, Intake right, 
        Elevator high, Elevator low, Drivetrain drive, XboxController controller, Climber climber){
        m_shooter = shooter;
        m_climber = climber;
        m_shooterHood = hood;
        m_turret = turret;
        m_leftIntake = left;
        m_rightIntake = right;
        m_lowElevator = low;
        m_highElevator = high;
        m_drive = drive;
        m_controller = controller;
        addRequirements(shooter, hood, turret, low, high , left, right, drive);

    }

    @Override
    public void initialize(){
        m_shooter.stop();
        m_turret.enable();
        m_turret.climbMode();
        m_shooterHood.run(1.0);
        m_leftIntake.stop();
        m_leftIntake.retract();
        m_rightIntake.stop();
        m_rightIntake.retract();
        m_lowElevator.stop();
        m_highElevator.stop();
        m_climber.extend();
        m_climber.changeConstraints(new Constraints(100, 250));
        m_climbModeReady = false;
    }

      /**
   * the execute function is overloaded with the function to drive the swerve drivetrain
   */
  @Override
  public void execute() {
    m_drive.drive(m_slewX.calculate(
        -inputTransform(m_controller.getLeftY()))
            * DriveConstants.kMaxSpeedMetersPerSecond/3.0,
        m_slewY.calculate(
          -inputTransform(m_controller.getLeftX()))
            * DriveConstants.kMaxSpeedMetersPerSecond/3.0,
        -inputTransform(m_controller.getRightX())
            * DriveConstants.kMaxAngularSpeed/3.0,
        true);

    if(m_turret.atSetpoint() && !m_climbModeReady){
        m_climbModeReady = true;
        m_climber.setDesiredPose(76.0);
        m_climber.run();
    }
    
  }

  @Override
  public void end(boolean interrupted){
    m_turret.disable();
    m_turret.stop();
    m_climber.setDesiredPose(2.0);
    m_climber.stop();
    m_climber.changeConstraints(new Constraints(50,25));
  }

  private double inputTransform(double input){
    return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  }

  public boolean isClimbModeReady(){
      return m_climbModeReady;
  }
}
