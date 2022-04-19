package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Utilities.AutoFromPathPlanner;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IndexElevator;
import frc.robot.commands.MoveFeed;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShootWhileMove;
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

public class FourBallAuto extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;
    private final Intake m_left;
    private final Intake m_right;

    public FourBallAuto(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color){

        m_drive = drivetrain;
        m_left = leftIntake;
        m_right = rightIntake;

        final AutoFromPathPlanner fourBallOne = new AutoFromPathPlanner(drivetrain, "20224BallAuto-1", 3.2);
        final AutoFromPathPlanner fourBallTwo = new AutoFromPathPlanner(drivetrain, "20224BallAuto-2", 3.2);
        final AutoFromPathPlanner fourBallThree = new AutoFromPathPlanner(drivetrain, "20224BallAuto-3", 3.2);
        

        final MoveFeed m_autoFeed = new MoveFeed(turret, top, bottom, drivetrain, shooter, hood, color);
        final MoveFeed m_autoFeed2 = new MoveFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(fourBallOne.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false, color),
                new SequentialCommandGroup(new WaitCommand(2.0),
                    fourBallOne.raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new RunIntake(leftIntake).alongWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop())))), 
                    fourBallTwo.andThen(new WaitCommand(1.0)).raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                    fourBallThree.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed2.raceWith(new RunIntake(rightIntake).raceWith( new WaitCommand(5.0).andThen(new InstantCommand(()->m_autoFeed2.stop()))))
        )));
    }
    
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
        m_left.retract();
        m_right.retract();
        m_left.stop();
        m_right.stop();
        
    }
}