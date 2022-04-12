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
import frc.robot.commands.ShootWhileMove;
import frc.robot.commands.ZeroClimb;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class Defense5n1 extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;

    public Defense5n1 (Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){

        m_drive = drivetrain;

        final AutoFromPathPlanner fiveoneUno = new AutoFromPathPlanner(drivetrain, "20225n1BallDefense-One", 3.2);
        final AutoFromPathPlanner fiveoneDos = new AutoFromPathPlanner(drivetrain, "20225n1BallDefense-Two", 3.2);
        final AutoFromPathPlanner fiveoneTres = new AutoFromPathPlanner(drivetrain, "20225n1BallDefense-Three", 3.2);
        final AutoFromPathPlanner fiveoneQuatro = new AutoFromPathPlanner(drivetrain, "20225n1BallDefense-Four", 3.2);
        final AutoFromPathPlanner fiveoneCinco = new AutoFromPathPlanner(drivetrain, "20225n1BallDefense-FIve", 3.2);

        final FeedShooter m_autoFeed = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed2 = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed3 = new FeedShooter(turret, top, bottom, drivetrain);

        RunShooter runShooter1 = new RunShooter(shooter, turret, drivetrain, hood, false);
        ShootWhileMove shootMove = new ShootWhileMove(shooter, turret, drivetrain, hood, false);
        ShootAtHanger hangShoot = new ShootAtHanger(shooter, turret, drivetrain, hood);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(fiveoneUno.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                runShooter1,
                new SequentialCommandGroup(
                    fiveoneUno.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop()).andThen(()->runShooter1.end(false))))    
                )
            ),
            new ParallelCommandGroup(
                new ShootAtHanger(shooter, turret, drivetrain, hood),
                new SequentialCommandGroup(
                    fiveoneDos.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    fiveoneTres,
                    m_autoFeed2.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed2.stop()).andThen(()->hangShoot.end(false))))
                
            )),

            new ParallelCommandGroup(
                shootMove,
                new SequentialCommandGroup(
                    fiveoneQuatro.raceWith(new RunIntake(leftIntake).alongWith(new IndexElevator(top, bottom))),
                    new RunIntake(leftIntake).raceWith(new WaitCommand(5.0)),
                    fiveoneCinco.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed3.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed3.stop()))) 
                
            )

                )
            );
    }
    
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
