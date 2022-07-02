package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.commands.Elevator.IndexElevator;
import frc.robot.commands.Elevator.SmartFeed;
import frc.robot.commands.Intakes.RunIntake;
import frc.robot.commands.TurretedShooter.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class FiveBall extends SequentialCommandGroup {

    private final Drivetrain m_drive;
    private final Intake m_left;
    private final Intake m_right;

    public FiveBall(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top,
            Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color) {

        m_drive = drivetrain;
        m_left = leftIntake;
        m_right = rightIntake;

        final AutoFromPathPlanner fiveBallUno = new AutoFromPathPlanner(drivetrain, "20225BallAuto-uno", 2.5);
        final AutoFromPathPlanner fiveBallDos = new AutoFromPathPlanner(drivetrain, "20225BallAuto-dos", 3.2);
        final AutoFromPathPlanner fiveBallTres = new AutoFromPathPlanner(drivetrain, "20225BallAuto-tres", 3.2);
        final AutoFromPathPlanner fiveBallQuatro = new AutoFromPathPlanner(drivetrain, "20225BallAuto-quatro", 3.2);

        final SmartFeed m_autoFeed = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final SmartFeed m_autoFeed2 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final SmartFeed m_autoFeed3 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(

                new InstantCommand(() -> drivetrain.resetOdometry(fiveBallUno.getInitialPose())),
                new InstantCommand(() -> climb.extend()),
                new ParallelCommandGroup(
                        new RunShooter(shooter, turret, drivetrain, hood, false, color),
                        new SequentialCommandGroup(
                                fiveBallUno
                                        .raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                                m_autoFeed.raceWith(new RunIntake(leftIntake).alongWith(
                                        new WaitCommand(1.0).andThen(new InstantCommand(() -> m_autoFeed.stop())))),
                                fiveBallDos
                                        .raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                                m_autoFeed2.raceWith(
                                        new WaitCommand(1.0).andThen(new InstantCommand(() -> m_autoFeed2.stop()))),
                                fiveBallTres.andThen(new WaitCommand(0.5))
                                        .raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                                fiveBallQuatro
                                        .raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                                m_autoFeed3.raceWith(new RunIntake(rightIntake).raceWith(
                                        new WaitCommand(5.0).andThen(new InstantCommand(() -> m_autoFeed3.stop())))))));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.updateKeepAngle();
        m_drive.stop();
        m_left.retract();
        m_right.retract();
        m_left.stop();
        m_right.stop();

    }
}
