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

public class OneBall extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;
    private final Intake m_left;
    private final Intake m_right;

    public OneBall(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb, ColorSensor color){

        m_drive = drivetrain;
        m_left = leftIntake;
        m_right = rightIntake;

        final AutoFromPathPlanner oneBallPath = new AutoFromPathPlanner(drivetrain, "20221Ball", 2.5);
        final MoveFeed m_autoFeed = new MoveFeed(turret, top, bottom, drivetrain, shooter, hood, color);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(oneBallPath.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false, color),
                new SequentialCommandGroup(
                    oneBallPath.raceWith((new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new WaitCommand(5.0).andThen(new InstantCommand(()->m_autoFeed.stop()))))));
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
