// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.JoystickAnalogButton;
import frc.robot.Utilities.JoystickAnalogButton.Side;
import frc.robot.commands.DriveByController;
import frc.robot.commands.InQueue;
import frc.robot.commands.Autos.AutoHangarOne;
import frc.robot.commands.Autos.AutoHangarTwo;
import frc.robot.commands.Autos.FiveBall;
import frc.robot.commands.Autos.FivePlusOne;
import frc.robot.commands.Autos.FivePlusTwo;
import frc.robot.commands.Autos.FourBallAuto;
import frc.robot.commands.Autos.OneBall;
import frc.robot.commands.Autos.TwoBallRight;
import frc.robot.commands.Climber.ClimbFromFloor;
import frc.robot.commands.Climber.Extend;
import frc.robot.commands.Climber.ClimbToHighBar;
import frc.robot.commands.Climber.FinalClimb;
import frc.robot.commands.Climber.InitiateClimbMode;
import frc.robot.commands.Climber.ZeroClimber;
import frc.robot.commands.Elevator.DemoFeed;
import frc.robot.commands.Elevator.IndexElevator;
import frc.robot.commands.Elevator.SmartFeed;
import frc.robot.commands.Intakes.RunIntake;
import frc.robot.commands.Intakes.SmartIntake;
import frc.robot.commands.Intakes.UnjamIntakes;
import frc.robot.commands.TurretedShooter.DemoShooter;
import frc.robot.commands.TurretedShooter.FaceTurret;
import frc.robot.commands.TurretedShooter.SmartShooter;
import frc.robot.commands.TurretedShooter.ZeroHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final Drivetrain m_robotDrive = new Drivetrain(); 

  private final Intake m_leftIntake = new Intake(IntakeConstants.kLeftMotorID, IntakeConstants.kLeftAirPorts, "Left");
  private final Intake m_rightIntake = new Intake(IntakeConstants.kRightMotorID, IntakeConstants.kRightAirPorts,
      "Right");

  private final Elevator m_lowElevator = new Elevator(ElevatorConstants.kLowMotorID, ElevatorConstants.kLowSensor,
      "Low");
  private final Elevator m_highElevator = new Elevator(ElevatorConstants.kHighMotorID, ElevatorConstants.kHighSensor,
      "High");

  private final Climber m_climber = new Climber();

  private final Turret m_turret = new Turret();

  private final Shooter m_shooter = new Shooter(ShooterConstants.kMotorIDs);

  private final ShooterHood m_hood = new ShooterHood();

  private final ColorSensor m_colorSensor = new ColorSensor();

  private final IndexElevator m_indexElevator = new IndexElevator(m_highElevator, m_lowElevator, m_colorSensor, m_shooter, m_hood, m_turret, m_operatorController);
  private final ZeroClimber m_ZeroClimb = new ZeroClimber(m_climber);
  private final ZeroHood m_ZeroHood = new ZeroHood(m_hood);

  private final SmartIntake m_runIntakes = new SmartIntake(m_rightIntake, m_leftIntake, m_robotDrive, m_driverController);

  private final UnjamIntakes m_unjamIntakes = new UnjamIntakes(m_leftIntake, m_rightIntake);

  private final InitiateClimbMode m_climbMode = new InitiateClimbMode(m_shooter, m_hood, m_turret, m_leftIntake,
      m_rightIntake, m_highElevator, m_lowElevator, m_climber);

  private final ClimbFromFloor m_firstClimb = new ClimbFromFloor(m_climber, m_robotDrive);
  private final ClimbToHighBar m_nextClimb = new ClimbToHighBar(m_climber, m_robotDrive);
  private final FinalClimb m_finalClimb = new FinalClimb(m_climber);
  private final Extend m_extend = new Extend(m_climber);

  private final SmartShooter m_moveShoot = new SmartShooter(m_shooter, m_turret, m_robotDrive, m_hood, true,
      m_colorSensor, m_driverController);

      //private final DemoShooter m_demo = new DemoShooter(m_shooter, m_turret, m_hood);

  private final SmartFeed m_moveFeed = new SmartFeed(m_turret, m_highElevator, m_lowElevator, m_robotDrive, m_shooter,
      m_hood, m_operatorController);

  private final DriveByController m_drive = new DriveByController(m_robotDrive, m_driverController);

  private final FaceTurret m_faceTurret = new FaceTurret(m_turret, m_robotDrive); 

  private final Command autoFiveTwo = new FivePlusTwo(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
    private final Command autoFiveOne = new FivePlusOne(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoFiveBall = new FiveBall(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoFourBall = new FourBallAuto(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoHangar1 = new AutoHangarOne(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoHangar2 = new AutoHangarTwo(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoTwoBallRight = new TwoBallRight(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);
  private final Command autoOneBall = new OneBall(m_robotDrive, m_leftIntake, m_rightIntake, m_lowElevator,
      m_highElevator, m_turret, m_hood, m_shooter, m_climber, m_colorSensor);

  private final Command doNothin = new WaitCommand(20.0);

  private final Command m_test = new InQueue(m_hood, m_climber, m_turret, m_lowElevator, m_highElevator, m_robotDrive);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoChooser();

    m_turret.setDefaultCommand(m_faceTurret);
    m_robotDrive.setDefaultCommand(m_drive);
    m_climber.setDefaultCommand(new RunCommand(() -> m_climber.run(), m_climber));
    m_hood.setDefaultCommand(new RunCommand(() -> m_hood.run(1.0), m_hood));
    m_lowElevator.setDefaultCommand(m_indexElevator);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(m_driverController, 0)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Rotation2d(0.0)));
    new JoystickAnalogButton(m_driverController, Side.kRight).whenPressed(m_runIntakes)
        .whenReleased(new InstantCommand(() -> m_runIntakes.cancel())
            .andThen(new WaitCommand(0.8).andThen(() -> m_rightIntake.stop()).andThen(() -> m_leftIntake.stop()))
            .withInterrupt(() -> m_runIntakes.isScheduled()));

    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_moveShoot);
    
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_moveShoot.cancel());

    //new JoystickButton(m_driverController, Button.kX.value).whenPressed(m_demo);
    
    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(3.89, 5.41, m_robotDrive.getGyro().times(-1.0))));

    new JoystickAnalogButton(m_operatorController, Side.kRight).whenPressed(m_moveFeed)
        .whenReleased(() -> m_moveFeed.stop());

    new JoystickAnalogButton(m_driverController, Side.kLeft).whenPressed(m_moveFeed)
        .whenReleased(() -> m_moveFeed.stop());

    new JoystickButton(m_driverController, Button.kRightBumper.value).whenHeld(m_unjamIntakes);
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenHeld(new RunIntake(m_leftIntake).alongWith(new RunIntake(m_rightIntake)))
        .whenReleased(new InstantCommand(()->m_leftIntake.stop()).andThen(new InstantCommand(()->m_rightIntake.stop())));

    new POVButton(m_operatorController, 0).whenPressed(m_ZeroHood);
    new POVButton(m_operatorController, 180).whenPressed(m_ZeroClimb);

    new JoystickButton(m_operatorController, Button.kBack.value).whenPressed(m_climbMode);
    new JoystickButton(m_operatorController, Button.kStart.value).whenPressed(() -> m_climbMode.cancel());

    new JoystickButton(m_operatorController, Button.kA.value)
        .whenPressed(new ConditionalCommand(m_firstClimb.andThen(new Extend(m_climber)), new WaitCommand(0.0), () -> m_climbMode.isScheduled()));
    new JoystickButton(m_operatorController, Button.kX.value)
        .whenPressed(new ConditionalCommand(m_extend, new WaitCommand(0.0), () -> m_climbMode.isScheduled()));    
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenPressed(new ConditionalCommand(m_finalClimb, new WaitCommand(0.0), () -> m_climbMode.isScheduled()));
    new JoystickButton(m_operatorController, Button.kY.value)
        .whenPressed(new ConditionalCommand(m_nextClimb.andThen(new Extend(m_climber)), new WaitCommand(0.0), () -> m_climbMode.isScheduled()));



  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Auto1Ball", autoOneBall);
    m_chooser.addOption("Auto2BallRight", autoTwoBallRight);
    m_chooser.addOption("Auto2Ball+1", autoHangar1);
    m_chooser.addOption("Auto2Ball+2", autoHangar2);
    m_chooser.addOption("Auto4Ball", autoFourBall);
    m_chooser.addOption("Auto5Ball", autoFiveBall);
    m_chooser.addOption("Auto5Ball+1", autoFiveOne);
    m_chooser.addOption("Auto5Ball+2", autoFiveTwo);
    m_chooser.addOption("Do Nothing", doNothin);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAuto() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the test command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getTest() {
    return m_test;
  }
  
  public void runTeleInitCommand() {
    if (getAuto() != null) {
      getAuto().cancel();
    }
    m_moveShoot.schedule();
    new WaitCommand(0.5).andThen(new SmartFeed(m_turret, m_highElevator, m_lowElevator, m_robotDrive, m_shooter, m_hood, m_colorSensor).withTimeout(2.5)).schedule();
  }
  
}
