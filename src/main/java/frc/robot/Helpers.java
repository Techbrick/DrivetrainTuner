package frc.robot;

import java.text.DecimalFormat;

public class Helpers {

    public static double  DeadbandJoystick(double value){
        double deadband = RobotMap.joystickDeadband;
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

}