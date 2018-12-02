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
public class FindMinDrivePower extends Command {
    
    private Robot _robot;
    private double minFrontDrive;
    private double minRearDrive;
    
    private Integer testPowerLevel;
    private Integer powerCounter;
    private Integer powerLevelTimer;
    private Integer PowerLevelTimeout;
    private boolean secondTurn = false;
    private boolean testCompleted;

  public FindMinDrivePower(Robot robot) {
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
    minFrontDrive = 0;
    minRearDrive = 0;
    
    SmartDashboard.putString("Status", "Determined Min FWD PWR: "+ Double.toString(testPowerLevel) + "%");
    SmartDashboard.putNumber("Min fwd PWR", minFrontDrive);
    SmartDashboard.putNumber("Min rvs Pwr", minRearDrive);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean moving = false;    
    double power = testPowerLevel;
    if(powerLevelTimer == 0){

        SmartDashboard.putString("Status", "Running power level: "+ Double.toString(testPowerLevel) + "%");
    }
    if(secondTurn){
        power = - power;
    }
    _robot.driveTrain.Move(power, power);
    double currentPosition = _robot.GetAverageEncoderPositionRaw();
    if(Math.abs(currentPosition) > 200){
        if(!secondTurn){
            minFrontDrive = testPowerLevel/100;
            SmartDashboard.putString("Status", "Determined Min FWD: "+ Double.toString(testPowerLevel) + "%");
            SmartDashboard.putNumber("Min fwd PWR", minFrontDrive);
        }else{
            minRearDrive = testPowerLevel/100;
            SmartDashboard.putString("Status", "Determined Min Right: "+ Double.toString(testPowerLevel) + "%");
            SmartDashboard.putNumber("Min rvs Pwr", minRearDrive);
            testCompleted = true;
        }
        moving = true;

    }  
    if(moving && !secondTurn){
        powerLevelTimer = 0;
        testPowerLevel = 2;
        secondTurn = true;

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
        if(minFrontDrive > minFrontDrive){
            RobotMap.minDrivePower = minFrontDrive;
        }else{
            RobotMap.minTurnPower = minFrontDrive;
        }
        SmartDashboard.putString("Status", "Determined Min Drive Power: "+ Double.toString(RobotMap.minDrivePower) + "%");
        return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
