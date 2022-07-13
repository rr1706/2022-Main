package frc.robot.commands.Autos;

import java.beans.IndexedPropertyChangeEvent;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.commands.Elevator.IndexElevator;
import frc.robot.commands.Elevator.SmartFeed;
import frc.robot.commands.Intakes.RunIntake;
import frc.robot.commands.TurretedShooter.SmartShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class FivePlusOne extends SequentialCommandGroup {

        private final Drivetrain m_drive;
        private final Intake m_left;
        private final Intake m_right;

    public FivePlusOne(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top,
            Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color) {

        m_drive = drivetrain;
        m_left = leftIntake;
        m_right = rightIntake;

        final AutoFromPathPlanner move11 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move1alt", 1.4, true);
        final AutoFromPathPlanner move12 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move1alt2", 1.4, true);
        final AutoFromPathPlanner move2 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move2", 1.8, true);

        final SmartFeed m_autoFeed = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color, 240.0);
        final SmartFeed m_autoFeed2 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color, 240.0);
        final SmartFeed m_autoFeed3 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color, 240.0);

        addCommands(

                new InstantCommand(() -> drivetrain.resetOdometry(move11.getInitialPose())),
                new InstantCommand(() -> climb.extend()),
                new ParallelCommandGroup(
                        new SmartShooter(shooter, turret, drivetrain, hood, true, color),
                        new SequentialCommandGroup(
                                move11.andThen(new WaitCommand(0.5),move12).raceWith(new RunIntake(leftIntake), new WaitCommand(2.25).raceWith(new IndexElevator(top, bottom)).andThen(m_autoFeed)),
                                move2.raceWith(new RunIntake(leftIntake), new RunIntake(rightIntake),m_autoFeed2),
                                new WaitCommand(10.0).raceWith(m_autoFeed3))
                                ));
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
