package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climber.ZeroClimber;
import frc.robot.commands.TurretedShooter.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class InQueue extends SequentialCommandGroup {
    private final Turret m_turret;

    public InQueue(ShooterHood hood, Climber climber, Turret turret, Elevator bottom, Elevator top, Drivetrain drive) {
        m_turret = turret;
        addCommands(
                new InstantCommand(() -> turret.stop()).alongWith(
                        new InstantCommand(() -> bottom.stop()).alongWith(new InstantCommand(() -> top.stop()))),
                new ZeroClimber(climber).alongWith(new ZeroHood(hood)), new InstantCommand(() -> climber.retract()),
                new RunCommand(() -> climber.stop(), climber).alongWith(new RunCommand(() -> hood.stop(), hood)));
        addRequirements(hood, climber, turret, bottom, drive);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

}
