package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.commands.Elevator.IndexElevator;
import frc.robot.commands.Elevator.SmartFeed;
import frc.robot.commands.Intakes.RunIntake;
import frc.robot.commands.TurretedShooter.RunShooter;
import frc.robot.commands.TurretedShooter.ShootAtHangar;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class AutoHangarOne extends SequentialCommandGroup {

    private final Drivetrain m_drive;

    public AutoHangarOne(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top,
            Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color) {

        m_drive = drivetrain;

        final AutoFromPathPlanner DefensivePt1 = new AutoFromPathPlanner(drivetrain, "20222BallAuto-uno", 3.2, true);
        final AutoFromPathPlanner DefensivePt2 = new AutoFromPathPlanner(drivetrain, "20222BallAuto-dos", 3.2, true);
        final AutoFromPathPlanner DefensivePt3 = new AutoFromPathPlanner(drivetrain, "20222BallPlus1-tres", 3.2, true);
        final AutoFromPathPlanner DefensivePt4 = new AutoFromPathPlanner(drivetrain, "20222BallPlus1-quatro", 3.2, true);

        final SmartFeed m_autoFeed = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final SmartFeed m_autoFeed2 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final SmartFeed m_autoFeed3 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(

                new InstantCommand(() -> drivetrain.resetOdometry(DefensivePt1.getInitialPose())),
                new InstantCommand(() -> climb.extend()),
                new ParallelRaceGroup(
                        new RunShooter(shooter, turret, drivetrain, hood, false, color),
                        new SequentialCommandGroup(
                                new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom, color, shooter, hood, turret))
                                        .raceWith(DefensivePt1.andThen(DefensivePt2)),
                                m_autoFeed.raceWith(
                                        new WaitCommand(1.5).andThen(new InstantCommand(() -> m_autoFeed.stop()))),
                                new InstantCommand(() -> rightIntake.stop()))),
                new ParallelRaceGroup(
                        new ShootAtHangar(shooter, turret, drivetrain, hood),
                        new SequentialCommandGroup(
                                DefensivePt3
                                        .raceWith(new IndexElevator(top, bottom, color, shooter, hood, turret).alongWith(new RunIntake(leftIntake))),
                                new WaitCommand(1.0).andThen(m_autoFeed2.raceWith(
                                        new WaitCommand(1.0).andThen(
                                                new InstantCommand(() -> m_autoFeed2.stop())))))),
                new ParallelRaceGroup(
                        new RunShooter(shooter, turret, drivetrain, hood, false, color),
                        new RunIntake(leftIntake),
                        new SequentialCommandGroup(
                                DefensivePt4.raceWith(new IndexElevator(top, bottom, color, shooter, hood, turret)),
                                m_autoFeed3.raceWith(
                                        new WaitCommand(5.0).andThen(new InstantCommand(() -> m_autoFeed3.stop()))))));
    }

    /*
    
    */
    @Override
    public void end(boolean interrupted) {
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
