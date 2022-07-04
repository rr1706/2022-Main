package frc.robot.commands.Elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class SmartFeed extends CommandBase {
    private final Turret m_turret;
    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Drivetrain m_drive;
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final XboxController m_operator;
    private final Timer m_timer = new Timer();
    private boolean m_End = false;
    private boolean m_ballShot = false;
    private boolean m_ballInThroat = false;
    private double m_shotTime = Double.NEGATIVE_INFINITY;
    private double m_throatTime = Double.NEGATIVE_INFINITY;
    private double m_shotDistLimiter = 240;

    public SmartFeed(Turret turret, Elevator top, Elevator bottom, Drivetrain drive, Shooter shooter, ShooterHood hood,
            XboxController operator) {
        m_turret = turret;
        m_top = top;
        m_bottom = bottom;
        m_drive = drive;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = operator;
        addRequirements(top, bottom);
    }

    public SmartFeed(Turret turret, Elevator top, Elevator bottom, Drivetrain drive, Shooter shooter, ShooterHood hood, ColorSensor color) {
m_turret = turret;
m_top = top;
m_bottom = bottom;
m_drive = drive;
m_shooter = shooter;
m_hood = hood;
m_operator = new XboxController(3);
addRequirements(top, bottom);
}

    public SmartFeed(Turret turret, Elevator top, Elevator bottom, Drivetrain drive, Shooter shooter, ShooterHood hood,
            ColorSensor color, double shotDistLimiter) {
        m_turret = turret;
        m_top = top;
        m_bottom = bottom;
        m_drive = drive;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = new XboxController(3);
        m_shotDistLimiter = shotDistLimiter;
        addRequirements(top, bottom);
    }

    @Override
    public void initialize() {
        m_top.stop();
        m_bottom.stop();
        m_End = false;
        m_timer.reset();
        m_timer.start();
        m_shotTime = Double.NEGATIVE_INFINITY;
        m_throatTime = Double.NEGATIVE_INFINITY;
        m_ballShot = false;
        m_ballInThroat = false;

    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();
        boolean isOmegaLow = Math.abs(m_drive.getFieldRelativeSpeed().omega) < Math.PI / 3.0;
        boolean isAlphaLow = Math.abs(m_drive.getFieldRelativeAccel().alpha) <= 1.00;
        boolean isTurretReady = m_turret.atDesiredAngle();
        boolean isShooterReady = m_shooter.atSetpoint();
        boolean isHoodReady = m_hood.atSetpoint();

        Translation2d robotToGoal = GoalConstants.kGoalLocation.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;
        boolean isClose = dist < m_shotDistLimiter;

        boolean[] array = { isOmegaLow, isAlphaLow, isTurretReady, isShooterReady, isHoodReady };

        SmartDashboard.putBooleanArray("Booleans", array);

        if (m_top.getSensor() && !m_ballInThroat) {
            m_throatTime = currentTime;
            m_ballInThroat = true;
        }

        boolean colorEvaluated = m_ballInThroat && currentTime >= m_throatTime + 0.100;

        SmartDashboard.putBoolean("color evaluated", colorEvaluated);
        SmartDashboard.putBoolean("ball in throat", m_ballInThroat);
        SmartDashboard.putBoolean("ball shot", m_ballShot);

        boolean canShoot = isOmegaLow && isAlphaLow && isTurretReady && isShooterReady && isHoodReady && isClose;
        if (canShoot && colorEvaluated && !m_ballShot) {
            m_shotTime = currentTime;
            m_ballShot = true;
            m_top.run();
        } else if (m_ballShot && currentTime >= m_shotTime + 0.060) {
            m_ballShot = false;
            m_ballInThroat = false;
        } else if (m_ballShot) {
            m_top.run();
        } else {
            if (m_top.getSensor() && m_bottom.getSensor()) {
                m_bottom.stop();
                m_top.stop();
                SmartDashboard.putBoolean("top index", true);
                SmartDashboard.putBoolean("bottom index", true);
                m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
                m_operator.setRumble(RumbleType.kRightRumble, 1.0);
            } else if (!m_bottom.getSensor() && !m_top.getSensor()) {
                m_bottom.run();
                m_top.run();
                SmartDashboard.putBoolean("top index", false);
                SmartDashboard.putBoolean("bottom index", false);
                m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
                m_operator.setRumble(RumbleType.kRightRumble, 0.0);
            } else if (m_top.getSensor() && !m_bottom.getSensor()) {
                m_bottom.run();
                m_top.stop();
                SmartDashboard.putBoolean("top index", true);
                SmartDashboard.putBoolean("bottom index", false);
                m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
                m_operator.setRumble(RumbleType.kRightRumble, 1.0);
            } else if (m_bottom.getSensor() && !m_top.getSensor()) {
                m_bottom.run();
                m_top.run();
                SmartDashboard.putBoolean("top index", false);
                SmartDashboard.putBoolean("bottom index", true);
                m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
                m_operator.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }

        SmartDashboard.putBoolean("Shooting", true);

    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_top.stop();
        m_bottom.stop();
        SmartDashboard.putBoolean("Shooting", false);
        m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
        m_operator.setRumble(RumbleType.kRightRumble, 0.0);
    }

    public void stop() {
        m_End = true;
    }

    @Override
    public boolean isFinished() {
        return m_End;
    }
}
