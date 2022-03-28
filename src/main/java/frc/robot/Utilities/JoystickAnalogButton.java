package frc.robot.Utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

public class JoystickAnalogButton extends Button {

  public static final class Side{
    public static final boolean kRight = false;
    public static final boolean kLeft = true;
  }

  XboxController m_controller;
  private double m_threshold = 0.25;
  private boolean m_side;

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param side Which side of the controller (Left = true, Right = false)
   */
  public JoystickAnalogButton(XboxController controller, boolean side) {
      m_controller = controller;
      m_side = side;
  }

  public boolean get() {
    if(m_side){
      return m_controller.getLeftTriggerAxis() > m_threshold; 
    }
    else{
      return m_controller.getRightTriggerAxis() > m_threshold; 
    }
  }

}