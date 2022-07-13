package frc.robot.commands.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class InitiateClimbMode extends CommandBase {
  private final Climber m_climber;
  private final Shooter m_shooter;
  private final ShooterHood m_shooterHood;
  private final Turret m_turret;
  private final Intake m_leftIntake;
  private final Intake m_rightIntake;
  private final Elevator m_highElevator;
  private final Elevator m_lowElevator;
  private boolean m_climbModeReady = false;

  public InitiateClimbMode(Shooter shooter, ShooterHood hood, Turret turret, Intake left, Intake right,
      Elevator high, Elevator low, Climber climber) {
    m_shooter = shooter;
    m_climber = climber;
    m_shooterHood = hood;
    m_turret = turret;
    m_leftIntake = left;
    m_rightIntake = right;
    m_lowElevator = low;
    m_highElevator = high;
    addRequirements(shooter, hood, turret, low, high, left, right);

  }

  @Override
  public void initialize() {
    m_shooter.stop();
    m_turret.setToStartPosition();
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
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {

    if (!m_climbModeReady) {
      m_climbModeReady = true;
      m_climber.setDesiredPose(69.5);
      m_climber.run();
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setToStartPosition();
    m_turret.stop();
    m_climber.extend();
    m_climber.setDesiredPose(2.0);
    m_climber.stop();
    m_climber.changeConstraints(ClimberConstants.kDefaultConstraints);
  }

  public boolean isClimbModeReady() {
    return m_climbModeReady;
  }
}
