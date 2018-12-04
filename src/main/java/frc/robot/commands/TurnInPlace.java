/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Stack;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DistancePid;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TurnPid;

/**
 * An example command.  You can replace me with your own command.
 */
public class TurnInPlace extends Command {
    
    private Robot _robot;
    private int stoppedCounter;
    private boolean testCompleted;
    private TurnPid _turnPid;
    private double _heading;
    

  public TurnInPlace(Robot robot, double heading) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
    _heading = heading;
    stoppedCounter = 0;
    
  }


  @Override
  protected void initialize() {
    
    
    SmartDashboard.putString("Instructions", "");
    SmartDashboard.putString("Status", "Running turn in place to " + Double.toString(_heading));
    testCompleted = false;
    // _robot.navX.reset();
    // _robot.navX.zeroYaw();
    stoppedCounter = 0;
    _turnPid = new TurnPid(RobotMap.kp_Angle, 0, 0, RobotMap.minTurnPower, .002, RobotMap.pidTurnDeadband);
    _turnPid.SetTargetAngle(_heading);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
      double power = _turnPid.GetAnglePidOutput(_robot.navX.getYaw());
      
      _robot.driveTrain.Move(-power, power); 
      if (power == 0){
          stoppedCounter ++;
          
      }else{
          stoppedCounter = 0;
          SmartDashboard.putNumber("test time", 0);
      }
      if (stoppedCounter > 5){
          testCompleted = true;
      }

   
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean done = testCompleted;
    if(done){
        
        SmartDashboard.putString("Status", "Completed turn in place to " + Double.toString(_heading));
        return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
    _robot.driveTrain.Move(0, 0); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
