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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class FindMinTurnPower extends Command {
    
    private Robot _robot;
    private double minLeftTurn;
    private double minRightTurn;
    private double avgTurnPower;
    private Integer testPowerLevel;
    private Integer powerCounter;
    private Integer powerLevelTimer;
    private Integer PowerLevelTimeout;
    private boolean secondTurn = false;
    private boolean testCompleted;

  public FindMinTurnPower(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
    


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.navX.reset();
    _robot.leftMaster.setSelectedSensorPosition(0, 0, 10);
    _robot.rightMaster.setSelectedSensorPosition(0, 0, 10);
    SmartDashboard.putString("Instructions", "The Robot will determine the min motor power to turn, press button 1 to end");
    SmartDashboard.putString("Status", "Running FindMinTurnPower");
    testPowerLevel = 2;
    powerCounter = 0;
    powerLevelTimer = 0;
    PowerLevelTimeout = 50;
    secondTurn = false;
    testCompleted = false;
    minRightTurn = 0;
    minLeftTurn = 0;
    avgTurnPower = 0;
    SmartDashboard.putString("Status", "Determined Min Left: "+ Double.toString(testPowerLevel) + "%");
    SmartDashboard.putNumber("Min left right power", minRightTurn);
    SmartDashboard.putNumber("Avg Min Turn Pwr", avgTurnPower);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean moving = false;    
    double power = testPowerLevel/100.0;
    if(powerLevelTimer == 0){

        SmartDashboard.putString("Status", "Running power level: "+ Double.toString(testPowerLevel) + "%");
    }
    if(secondTurn){
        power = - power;
    }
    _robot.driveTrain.Move(power, -power);
    int currentPosition = _robot.leftMaster.getSelectedSensorPosition(0);
    if(Math.abs(currentPosition) > 200){
        if(!secondTurn){
            minLeftTurn = testPowerLevel/100.0;
            SmartDashboard.putString("Status", "Determined Min Left: "+ Double.toString(testPowerLevel) + "%");
            SmartDashboard.putNumber("Min left turn power", minLeftTurn);
        }else{
            minRightTurn = testPowerLevel/100.0;
            SmartDashboard.putString("Status", "Determined Min Right: "+ Double.toString(testPowerLevel) + "%");
            SmartDashboard.putNumber("Min left right power", minRightTurn);
            testCompleted = true;
        }
        moving = true;

    }  
    if(moving && !secondTurn){
        powerLevelTimer = 0;
        testPowerLevel = 2;
        secondTurn = true;
        _robot.leftMaster.setSelectedSensorPosition(0, 0, 10);
    }else if(!moving){

        powerLevelTimer ++;
        if(powerLevelTimer > PowerLevelTimeout){
            testPowerLevel = testPowerLevel + 2;
            powerLevelTimer = 0;
        }
    }      
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean done = _robot.stick.getRawButton(1) || testCompleted;
    if(done){
        if(minLeftTurn > minRightTurn){
            RobotMap.minTurnPower = minLeftTurn;
        }else{
            RobotMap.minTurnPower = minRightTurn;
        }
        SmartDashboard.putString("Status", "Determined Min Turn Power: "+ Double.toString(RobotMap.minTurnPower) + "%");
        _robot.driveTrain.Move(0.0,0.0);
        return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
    _robot.driveTrain.Move(0.0,0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
