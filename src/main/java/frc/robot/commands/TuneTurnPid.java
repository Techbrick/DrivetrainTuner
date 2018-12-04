/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.PipedReader;
import java.util.ArrayList;
import java.util.Stack;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TurnPid;
import frc.robot.DTO.PidDataDTO;

/**
 * An example command.  You can replace me with your own command.
 */
public class TuneTurnPid extends Command {
    
    private Robot _robot;
   
    
    private Integer powerCounter;
    private Integer turnTimer;
    private Integer PowerLevelTimeout;
    private boolean secondTurn = false;
    private boolean testCompleted;
    private TurnPid _turnPid;
    Timer _timer;
    double _startTime;
    private int stoppedTimer;
    private double testKp;
    private double lastPower;
    private int iterationCounter;
    private int oscilationCounter;
    private int stoppedCounter;
    private ArrayList<PidDataDTO> resultsArray;
    private double pIncrement;
    private double bestTime;
    boolean moving = false;    
    double target = 0;
    double testTime = 0;

  public TuneTurnPid(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
    


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.navX.reset();
    _robot.navX.zeroYaw();
    _robot.leftMaster.setSelectedSensorPosition(0, 0, 10);
    _robot.rightMaster.setSelectedSensorPosition(0, 0, 10);
    SmartDashboard.putString("Instructions", "The Robot iterate through potential Kp values and will determine the best one, hold 1 to run, press button 2 to end");
    SmartDashboard.putString("Status", "Running Tune Turn Pid");
    oscilationCounter = 0;
    powerCounter = 0;
    turnTimer = 0;
    PowerLevelTimeout = 250;
    secondTurn = false;
    testCompleted = false;
    testKp = RobotMap.minTurnPower/40;
    iterationCounter = 0;
    SmartDashboard.putNumber("Testing Kp angle", testKp);
    resultsArray = new ArrayList<PidDataDTO>();
    pIncrement = testKp/10;
    bestTime = 1000;
    _timer = new Timer();
    secondTurn = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    
    if(_robot.stick.getRawButton(1)){
        if(iterationCounter % 2 == 0){
            target = 90;
            secondTurn = true;
        }
        if(turnTimer == 0){
            _turnPid = new TurnPid(testKp, 0, 0, RobotMap.minTurnPower, .002, 2);
            _turnPid.SetTargetAngle(target);
            SmartDashboard.putString("Status", "RunningKp: "+ Double.toString(testKp) + " target: " + Double.toString(target) + " test #" + Integer.toString(iterationCounter));
            SmartDashboard.putNumber("Testing Kp angle", testKp);
            _timer.reset();
            _timer.start();
            oscilationCounter = 0;
            stoppedTimer = 0;
        }
        double power = _turnPid.GetAnglePidOutput(_robot.navX.getYaw());
        _robot.driveTrain.Move(-power, power);
        // compare lastPower to current to check for sign flipped;
        if(lastPower*power < 0){
            oscilationCounter ++;
        }
        stoppedTimer++;
        if (power == 0){
            stoppedCounter ++;
            if(stoppedCounter == 1){
                testTime = _timer.get();
            SmartDashboard.putNumber("test time", _timer.get());
            }
        }else{
            stoppedCounter = 0;
            SmartDashboard.putNumber("test time", 0);
        }
        if (stoppedCounter > 25){
            PidDataDTO pidResult = new PidDataDTO();
            pidResult.Kp = testKp;
            pidResult.time = testTime;
            resultsArray.add(pidResult);
            turnTimer =0;
            iterationCounter ++;
            testKp = testKp + pIncrement;
            if(testTime < bestTime){
                SmartDashboard.putNumber("Tune TurnPid Best Time", testTime);
                bestTime = testTime;
            }else{
                double timeDif = Math.abs(bestTime - testTime);
                if(timeDif < 50){
                    testCompleted = true;
                }
            }
            
        }
        if(oscilationCounter > 2){
            PidDataDTO pidResult = new PidDataDTO();
            pidResult.Kp = testKp;
            pidResult.time = 999;
            resultsArray.add(pidResult);
            turnTimer =0;
            iterationCounter ++;
            testKp = testKp - pIncrement;
            pIncrement = pIncrement/10;
            testKp = testKp + pIncrement;
        }
        if(stoppedTimer > PowerLevelTimeout){
            PidDataDTO pidResult = new PidDataDTO();
            pidResult.Kp = testKp;
            pidResult.time = 999;
            resultsArray.add(pidResult);
            turnTimer =0;
            iterationCounter ++;
            testKp = testKp + pIncrement;
            
        }
    }else{
        _robot.driveTrain.Move(0,0);
    }
    

    

    
    
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean done = _robot.stick.getRawButton(2) || testCompleted;
    if(done){
        
        SmartDashboard.putString("Status", "Determined best Turn Kp: "+ Double.toString(testKp));
        _robot.driveTrain.Move(0,0);
        return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _robot.driveTrain.Move(0,0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}
