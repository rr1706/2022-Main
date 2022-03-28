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

public class FiveBall extends SequentialCommandGroup{
 
    private final Drivetrain m_drive;

    public FiveBall(Drivetrain drivetrain, Intake leftIntake, Intake rightIntake, Elevator bottom, Elevator top, Turret turret, ShooterHood hood, Shooter shooter, Climber climb){

        m_drive = drivetrain;

        final AutoFromPathPlanner fiveBallUno = new AutoFromPathPlanner(drivetrain, "20225BallAuto-uno", 3.2);
        final AutoFromPathPlanner fiveBallDos = new AutoFromPathPlanner(drivetrain, "20225BallAuto-dos", 3.2);
        final AutoFromPathPlanner fiveBallTres = new AutoFromPathPlanner(drivetrain, "20225BallAuto-tres", 3.2);
        final AutoFromPathPlanner fiveBallQuatro = new AutoFromPathPlanner(drivetrain, "20225BallAuto-quatro", 3.2);

        final FeedShooter m_autoFeed = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed2 = new FeedShooter(turret, top, bottom, drivetrain);
        final FeedShooter m_autoFeed3 = new FeedShooter(turret, top, bottom, drivetrain);

        addCommands(
            
            new InstantCommand(()->drivetrain.resetOdometry(fiveBallUno.getInitialPose())),
            new InstantCommand(()->climb.extend()),
            new ParallelCommandGroup(
                new RunShooter(shooter, turret, drivetrain, hood, false),
                new SequentialCommandGroup(
                    fiveBallUno.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
                    m_autoFeed.raceWith(new WaitCommand(1.0).andThen(new InstantCommand(()->m_autoFeed.stop()))), 
                    fiveBallDos.raceWith(new RunIntake(rightIntake).alongWith(new IndexElevator(top, bottom))),
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
