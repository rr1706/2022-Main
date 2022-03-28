package frc.robot.Utilities;

import frc.robot.Constants.*;

/**
 * This file is created to hold any particularly useful math utilies which may
 * be called statically
 */
public class MathUtils {
    //Converts a radian angle into the unit circle range of 0 to 2PI
    /**
    * This function takes in an angle in radians and does the proper math to convert the output to an angle from 0 to 2PI
   * also known as a "unit circle angle"
    *
    * @param angle is the input angle in radians
    * @return a unit circle angle equal to the input angle 
    */
    public static double toUnitCircAngle(double angle){
        double rotations = angle / (2*Math.PI);        
        return (angle - Math.round(rotations-0.500)*Math.PI*2.0);
    }

    /**
    * Takes an input value and squares it, but retains the sign. IE negative numbers will remain negative.
    @param input is the number to perform the transform on
    @return the transformed input value
    */
    public static double singedSquare(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }
    /**
    * Applies a simple deadband to input values between -1.0 and 1.0. Makes use of the deadband constants stored in the constants
    * class. Values below the innerDeadband will be set to zero. Values above the outerDeadband will be set to 1.0 or -1.0 depending
    * on the sign of the input
    @param input is the input to perform the deadband on
    @return the resulting input after applying the deadband
    */
    public static double applyDeadband(double input) {
        if (Math.abs(input) < DriveConstants.kInnerDeadband) {
          return 0.0;
        } else if (Math.abs(input) > DriveConstants.kOuterDeadband) {
          return Math.signum(input) * 1.0;
        } else {
          return input;
        }
      }
    
}
