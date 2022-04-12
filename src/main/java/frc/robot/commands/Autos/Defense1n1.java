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
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShootAtHanger;
import frc.robot.commands.ZeroClimb;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class Defense1n1 extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;

    public Defense1n1 (Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){

        m_drive = drivetrain;

        final AutoFromPathPlanner unounoUno = new AutoFromPathPlanner(drivetrain, "20221n1BallDefense-One", 3.2);
        final AutoFromPathPlanner unounoDos = new AutoFromPathPlanner(drivetrain, "20221n1BallDefense-Two", 3.2);

        final FeedShooter m_autoFeed = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed2 = new FeedShooter(turret, top, bottom, drivetrain);

        RunShooter runShooter = new RunShooter(shooter, turret, drivetrain, hood, false);
        ShootAtHanger hangShoot = new ShootAtHanger(shooter, turret, drivetrain, hood);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(unounoUno.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                runShooter,
                new SequentialCommandGroup(
                    m_autoFeed.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop())),
                    unounoUno.raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom)))
                ),

            new ParallelCommandGroup(
                hangShoot,
                new SequentialCommandGroup(
                    unounoDos,
                    m_autoFeed2.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed2.stop())))))    
                )
            ));
            
    }
    
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
