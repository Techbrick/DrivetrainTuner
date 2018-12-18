/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveEncoderCal extends Command {
    private double startingEncoderPosition;
    private double endingEncoderPosition;
    private Robot _robot;

  public DriveEncoderCal(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startingEncoderPosition = _robot.driveTrain.GetAverageEncoderPositionRaw();
    SmartDashboard.putString("Instructions", "Drive forward exactly 48 inches and when stoped press button 1");
    SmartDashboard.putString("Status", "Running Encoder Cal");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(_robot == null){
        DriverStation.reportError("_robot is null", false);
        SmartDashboard.putString("Status", "_robot is null");
    }else{
       // SmartDashboard.putString("Status", "Running Encoder Cal execute");

        double power = Helpers.DeadbandJoystick(_robot.stick.getY(), _robot.robotMap);
        SmartDashboard.putString("Status", "Running Encoder Cal execute stick "+ Double.toString(power));
        _robot.driveTrain.Move(power, power);
    }
      
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean done = _robot.stick.getRawButton(1);
    if(done){
        
        SmartDashboard.putString("Status", "Calculating Encoder Cal");
        endingEncoderPosition = _robot.driveTrain.GetAverageEncoderPositionRaw();
    
        return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Status", "Calculating Encoder Cal");
    double ticksPerInch = (endingEncoderPosition - startingEncoderPosition)/48;
    
    if(ticksPerInch == 0){
        SmartDashboard.putString("Instructions", "Your should check the connections and configuration of your encoders");
        SmartDashboard.putString("Status", "ERROR - There was no change in encoder position detected");
    }else{
        SmartDashboard.putString("Instructions", "Your should update your stored contants to the new value");
        SmartDashboard.putString("Status", "Calculated Encoder Cal: " + String.valueOf( ticksPerInch ) + "ticks per inch");
        SmartDashboard.putNumber("Endcoder Ticks/In", ticksPerInch);
        _robot.encoderConstant =1/ticksPerInch;
    }
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    SmartDashboard.putString("Status", "Calculating Encoder Cal");
    endingEncoderPosition = _robot.driveTrain.GetAverageEncoderPositionRaw();
    
    double ticksPerInch = (endingEncoderPosition - startingEncoderPosition)/48;
    
    if(ticksPerInch == 0){
        SmartDashboard.putString("Instructions", "Your should check the connections and configuration of your encoders");
        SmartDashboard.putString("Status", "ERROR - There was no change in encoder position detected");
    }else{
        SmartDashboard.putString("Instructions", "Your should update your stored contants to the new value");
        SmartDashboard.putString("Status", "Calculated Int Encoder Cal: " + String.valueOf( ticksPerInch ) + "ticks per inch");
        SmartDashboard.putNumber("Endcoder Ticks/In", ticksPerInch);
        _robot.encoderConstant =1/ticksPerInch;
    }
  }
}
