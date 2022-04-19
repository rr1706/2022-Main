package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.MoveFeed;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShootAtHanger;
import frc.robot.commands.ZeroClimb;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class AutoHangarTwo extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;

    public AutoHangarTwo(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color){

        m_drive = drivetrain;

        final AutoFromPathPlanner DefensivePt1 = new AutoFromPathPlanner(drivetrain, "20222BallAuto-uno", 3.2);
        final AutoFromPathPlanner DefensivePt2 = new AutoFromPathPlanner(drivetrain, "20222BallAuto-dos", 3.2);
        final AutoFromPathPlanner DefensivePt3 = new AutoFromPathPlanner(drivetrain, "20222BallPlus2-tres", 3.2);


        final MoveFeed m_autoFeed = new MoveFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final MoveFeed m_autoFeed2 = new MoveFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(DefensivePt1.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelRaceGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false, color),
                new SequentialCommandGroup(
                    new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom)).raceWith(DefensivePt1.andThen(DefensivePt2)),
                    m_autoFeed.raceWith(new WaitCommand(1.5).andThen(new InstantCommand(()->m_autoFeed.stop()))),
                    new InstantCommand(() -> rightIntake.stop()))),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    DefensivePt3.raceWith(new IndexElevator(top, bottom).alongWith(new RunIntake(leftIntake).raceWith(new WaitCommand(1)).andThen(new RunIntake(rightIntake)))),
                    new ShootAtHanger(shooter, turret, drivetrain, hood).alongWith(new WaitCommand(1.0).andThen(m_autoFeed2.raceWith(
                        new WaitCommand(1.0).andThen(
                            new InstantCommand(()->m_autoFeed2.stop())))))
                    
                    )
                            
                )
        );
    }
    /*
    
    */
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
