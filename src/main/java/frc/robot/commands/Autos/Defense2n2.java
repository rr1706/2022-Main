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
import frc.robot.commands.ZeroClimb;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Swerve.Drivetrain;

public class Defense2n2 extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;

    public Defense2n2 (Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){

        m_drive = drivetrain;

        final AutoFromPathPlanner twotwoUno = new AutoFromPathPlanner(drivetrain, "20222n2BallDefense-One", 3.2);
        final AutoFromPathPlanner twotwoDos = new AutoFromPathPlanner(drivetrain, "20222n2BallDefense-Two", 3.2);
        final AutoFromPathPlanner twotwoTres = new AutoFromPathPlanner(drivetrain, "20222n2BallDefense-Three", 3.2);
        final AutoFromPathPlanner twotwoQuatro = new AutoFromPathPlanner(drivetrain, "20225BallDefense-Four", 3.2);
        final AutoFromPathPlanner twotwoCinco = new AutoFromPathPlanner(drivetrain, "20225BallDefense-Five", 3.2);

        final FeedShooter m_autoFeed = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed2 = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed3 = new FeedShooter(turret, top, bottom, drivetrain);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(fiveBallUno.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false),
                new SequentialCommandGroup(
                    twotwoUno.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop()))), 
                    twotwoDos.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed2.raceWith(new WaitCommand(1.5).andThen(new InstantCommand(()->m_autoFeed2.stop()))),
                    fiveBallTres.andThen(new WaitCommand(1.0)).raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    fiveBallQuatro.raceWith(new IndexElevator(top, bottom)),
                    m_autoFeed3.raceWith(new WaitCommand(5.0).andThen(new InstantCommand(()->m_autoFeed3.stop())))
        )));
    }
    
    @Override
    public void end(boolean interrupted){
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
