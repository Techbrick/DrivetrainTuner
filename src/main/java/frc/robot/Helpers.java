package frc.robot;

import java.text.DecimalFormat;

public class Helpers {

    public static double  DeadbandJoystick(double value, RobotMap robotMap){
        double deadband = robotMap.joystickDeadband;
        if (value >= deadband) 
        return value;
    
        /* Lower deadband */
        if (value <= -deadband)
            return value;
        
        /* Outside deadband */
        return 0;

    }
    public static String DblTo2PLaces(double input) {
        DecimalFormat decimalFormat = new DecimalFormat("#,##0.00");
        String numberAsString = decimalFormat.format(input);
        return numberAsString;
    }
    public static String DblTo1PLace(double input) {
        DecimalFormat decimalFormat = new DecimalFormat("#,##0.00");
        String numberAsString = decimalFormat.format(input);
        return numberAsString;
    }
    public static double ConvertYawToHeading(double yaw){
        if(yaw < 0){
            yaw = yaw +360;
        }
        return yaw;
    }
    public static double FeetToMeters(double fps){
        return fps*.3048;
    }
    public static double InchesToMeters(double inches){
        return inches /12*.3048;
    }

}