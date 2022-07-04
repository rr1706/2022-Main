package frc.robot.commands.Autos;

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

public class FivePlusSweep extends SequentialCommandGroup {

        private final Drivetrain m_drive;
        private final Intake m_left;
        private final Intake m_right;

    public FivePlusSweep(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top,
            Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color) {

        m_drive = drivetrain;
        m_left = leftIntake;
        m_right = rightIntake;

        final AutoFromPathPlanner move1 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move1", 2.5, false);
        final AutoFromPathPlanner move2 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move2", 2.0, false);
        final AutoFromPathPlanner move3 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move3", 1.0, false);
        final AutoFromPathPlanner move4 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move4", 3.2, true);
        final AutoFromPathPlanner move5 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move5", 3.2, false);
        final AutoFromPathPlanner move6 = new AutoFromPathPlanner(drivetrain, "20225BallAuto-Move6", 1.5, true);

        final SmartFeed m_autoFeed = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final SmartFeed m_autoFeed2 = new SmartFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(

                new InstantCommand(() -> drivetrain.resetOdometry(move1.getInitialPose())),
                new InstantCommand(() -> climb.extend()),
                new ParallelCommandGroup(
                        new SmartShooter(shooter, turret, drivetrain, hood, true, color),
                        new SequentialCommandGroup(
                                move1.raceWith(new RunIntake(leftIntake), new IndexElevator(top, bottom)),
                                new ParallelRaceGroup(move2.andThen(move3), new RunIntake(leftIntake), m_autoFeed), 
                                move4.andThen(new WaitCommand(0.5)).raceWith(new RunIntake(leftIntake),new IndexElevator(top, bottom)),
                                new InstantCommand(() -> leftIntake.stop()),
                                new ParallelRaceGroup(move5.andThen(move6).andThen(
                                        new WaitCommand(10.0)), new RunIntake(rightIntake), m_autoFeed2))));
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
