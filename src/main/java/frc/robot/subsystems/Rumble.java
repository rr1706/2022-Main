package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {
    private final XboxController m_controller;
    private final Timer m_timer = new Timer();
    
    private double m_time = 0.0;
    private int m_temp = 0;
    private int m_side = 0;
    private double m_strength = 0.0;
    private String m_type = "Continuous";
    private int m_rumbling = 0;

    public Rumble(XboxController controller) {
        m_controller = controller;
        m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
        m_controller.setRumble(RumbleType.kRightRumble, 0.0);
        m_timer.stop();
        m_timer.reset();
    }

    /**
     * Set the rumble in the controller
     * @param side 0 = Both - 1 = Left - 2 = Right
     * @param strength Strength of the rumble
     * @param time Length of time to rumble
     */
    public void setRumble(int side, double strength, double time) {
        m_strength = strength;
        m_type = "Continous";
        m_side = side;
        m_time = time;
        m_rumbling = 1;
        m_timer.reset();
        m_timer.start();

        if (m_side == 1) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
        } else if (m_side == 2) {
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        } else if (m_side == 0) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        }
    }

    /**
     * Set the rumble in the controller
     * @param pattern Pulse, Tap, Flutter, Continuous
     * @param side 0 = Both - 1 = Left - 2 = Right
     * @param strength Strength of the rumble
     * @param time Length of time to rumble
     */
    public void setRumble(String pattern, int side, double strength, double time) {
        m_strength = strength;
        m_type = pattern;
        m_side = side;
        m_time = time;
        m_rumbling = 1;
        m_timer.reset();
        m_timer.start();
        
        if (m_side == 1) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
        } else if (m_side == 2) {
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        } else if (m_side == 0) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        }
    }

    /**
     * Set the rumble in the controller
     * @param side 0 = Both - 1 = Left - 2 = Right
     * @param strength Strength of the rumble
     */
    public void toggleRumble(int side, double strength) {
        m_strength = strength;
        m_type = "Continous";
        m_side = side;
        m_rumbling = (int) Math.cos(m_rumbling * Math.PI / 2);;
        m_timer.reset();
        m_timer.start();

        if (m_side == 1) {
            m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * strength);
        } else if (m_side == 2) {
            m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * strength);
        } else if (m_side == 0) {
            m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * strength);
            m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * strength);
        }
    }

    /**
     * Set the rumble in the controller
     * @param pattern Pulse, Tap, Flutter, Continuous
     * @param side 0 = Both - 1 = Left - 2 = Right
     * @param strength Strength of the rumble
     */
    public void toggleRumble(String pattern, int side, double strength) {
        m_strength = strength;
        m_type = pattern;
        m_side = side;
        m_rumbling = (int) Math.cos(m_rumbling * Math.PI / 2);
        m_timer.reset();
        m_timer.start();
        
        if (m_side == 1) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
        } else if (m_side == 2) {
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        } else if (m_side == 0) {
            m_controller.setRumble(RumbleType.kLeftRumble, strength);
            m_controller.setRumble(RumbleType.kRightRumble, strength);
        }
    }

    @Override
    public void periodic() {
        if (m_time != 0.0 && m_timer.hasElapsed(m_time)) {
            m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
            m_controller.setRumble(RumbleType.kRightRumble, 0.0);
            m_timer.stop();
            m_timer.reset();
            m_time = 0.0;
            m_strength = 0.0;
            m_type = "Continous";
            m_rumbling = 0;
        }

        switch(m_type) {
            case "Pulse":
                if (m_time % 0.8 == 0) {
                    if (m_side == 0) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    } else if (m_side == 1) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                    } else if (m_side == 2) {
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    }
                    m_rumbling = (int) Math.cos(m_rumbling * Math.PI / 2);
                }
            break;
            case "Tap":
                if (m_time % 0.4 == 0) {
                    if (m_side == 0) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    } else if (m_side == 1) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                    } else if (m_side == 2) {
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    }
                    m_rumbling = (int) Math.cos(m_rumbling * Math.PI / 2);
                }
            break;
            case "Flutter":
                if (m_time % 0.2 == 0) {
                    if (m_side == 0) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    } else if (m_side == 1) {
                        m_controller.setRumble(RumbleType.kLeftRumble, m_rumbling * m_strength);
                    } else if (m_side == 2) {
                        m_controller.setRumble(RumbleType.kRightRumble, m_rumbling * m_strength);
                    }
                    m_rumbling = (int) Math.cos(m_rumbling * Math.PI / 2);
            }
            break;
        }

    }

}
