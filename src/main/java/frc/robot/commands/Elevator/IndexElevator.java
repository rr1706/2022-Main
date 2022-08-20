package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class IndexElevator extends CommandBase {

    private final Elevator m_top;
    private final Elevator m_bottom;
    private final Turret m_turret;
    private final Shooter m_shooter;
    private final ShooterHood m_hood;
    private final XboxController m_operator;
    private final ColorSensor m_color;

    public IndexElevator(Elevator top, Elevator bottom, ColorSensor color, Shooter shooter, ShooterHood hood, Turret turret, XboxController operator) {
        m_top = top;
        m_bottom = bottom;
        m_operator = operator;
        m_color = color;
        m_turret = turret;
        m_shooter = shooter;
        m_hood = hood;
        addRequirements(bottom);
    }

    public IndexElevator(Elevator top, Elevator bottom, ColorSensor color, Shooter shooter, ShooterHood hood, Turret turret) {
        m_top = top;
        m_bottom = bottom;
        m_color = color;
        m_turret = turret;
        m_shooter = shooter;
        m_hood = hood;
        m_operator = new XboxController(3);
        addRequirements(bottom);
    }

    @Override
    public void execute() {

        boolean okayToSpit = m_turret.atDesiredAngle() && m_shooter.atSetpoint() && m_hood.atSetpoint() && m_color.isWrongBall();

        if(okayToSpit){
            m_top.run();
            m_bottom.stop();
        } else if (m_top.getSensor() && m_bottom.getSensor()) {
            m_bottom.stop();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", true);
            //m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
            //m_operator.setRumble(RumbleType.kRightRumble, 1.0);
        } else if (!m_bottom.getSensor() && !m_top.getSensor()) {
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", false);
            //m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
            //m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        } else if (m_top.getSensor() && !m_bottom.getSensor()) {
            m_bottom.run();
            m_top.stop();
            SmartDashboard.putBoolean("top index", true);
            SmartDashboard.putBoolean("bottom index", false);
            //m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
            //m_operator.setRumble(RumbleType.kRightRumble, 1.0);
        } else if (m_bottom.getSensor() && !m_top.getSensor()) {
            m_bottom.run();
            m_top.run();
            SmartDashboard.putBoolean("top index", false);
            SmartDashboard.putBoolean("bottom index", true);
            //m_operator.setRumble(RumbleType.kLeftRumble, 1.0);
            //m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_operator.setRumble(RumbleType.kLeftRumble, 0.0);
        m_operator.setRumble(RumbleType.kRightRumble, 0.0);
        m_bottom.stop();
        m_top.stop();
    }
}
