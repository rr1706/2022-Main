package frc.robot.commands.Autos;

import java.nio.file.attribute.DosFileAttributeView;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class TwoBallOne extends SequentialCommandGroup {
    private final Drivetrain m_drive;

    public TwoBallOne(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){
        final AutoFromPathPlanner twoBallPath = new AutoFromPathPlanner(drivetrain, "20222BallAuto-one", 3.2);
        final FeedShooter m_autoFeed = new FeedShooter(turret, top, bottom, drivetrain);

        m_drive = drivetrain;

        addCommands(
            new InstantCommand(()->drivetrain.resetOdometry(twoBallPath.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false),
                new SequentialCommandGroup(
                    twoBallPath.raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new WaitCommand(2.0).andThen(new InstantCommand(()->m_autoFeed.stop())))))
        );

        
    }
        
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
    }

}