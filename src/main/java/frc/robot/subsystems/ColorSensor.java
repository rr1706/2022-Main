package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.215, 0.434, 0.350);
    private final Color kRedTarget = new Color(0.335, 0.421, 0.242);
    private final Color kNoTarget = new Color(0.256, 0.457, 0.287);

    private ColorMatchResult m_match;

    public ColorSensor(){
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kNoTarget);

    }

    public double[] getBallColor() {
        double[] ret = { m_colorSensor.getRed(), m_colorSensor.getGreen(), m_colorSensor.getBlue() };
        return ret;
    }

    @Override
    public void periodic() {

        Color detectedColor = m_colorSensor.getColor();

        m_match = m_colorMatcher.matchClosestColor(detectedColor);

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", m_match.confidence);
        SmartDashboard.putNumber("Prox", m_colorSensor.getProximity());

        if(m_match.color == kRedTarget){
            SmartDashboard.putString("Detected Color", "Red");
        }
        else if(m_match.color == kBlueTarget){
            SmartDashboard.putString("Detected Color", "Blue");
        }
        else if(m_match.color == kNoTarget){
            SmartDashboard.putString("Detected Color", "Neither");
        }

    }

public boolean isWrongBall(){
    boolean prox = m_colorSensor.getProximity() >= 80;
    boolean red = m_match.color == kRedTarget;
    boolean blue = m_match.color == kBlueTarget;
    if(DriverStation.getAlliance().equals(Alliance.Red) && blue && prox){
        return true;
    }
    else if(DriverStation.getAlliance().equals(Alliance.Blue) && red && prox){
        return true;
    }
    else{
        return false;
    }
}

}