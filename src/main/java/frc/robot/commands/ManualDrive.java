/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ManualDrive extends Command {
  private Robot _robot;

  public ManualDrive(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
        
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = Helpers.DeadbandJoystick(_robot.stick.getY());
    double twist =Helpers.DeadbandJoystick( _robot.stick.getTwist()/2);
    _robot.driveTrain.ArcadeDrive(power, twist);    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Instructions", "DONE With sample auto");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
