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
import frc.robot.DistancePid;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;

import frc.robot.DTO.PidDataDTO;

/**
 * An example command.  You can replace me with your own command.
 */
public class TuneDistancePid extends Command {
    
    private Robot _robot;
   
    
    private Integer powerCounter;
    private Integer turnTimer;
    private Integer PowerLevelTimeout;
    private boolean secondTurn = false;
    private boolean testCompleted;
    private DistancePid _distPid;
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
    double loopStart;
    double loopEnd;
    double testTime = 0;

  public TuneDistancePid(Robot robot) {
    // Use requires() here to declare subsystem dependencies
    _robot = robot;
    requires(_robot.driveTrain);
    


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _robot.navX.reset();
    _robot.navX.zeroYaw();
    _robot.driveTrain.ResetEncoders();
    SmartDashboard.putString("Instructions", "The Robot iterate through potential Kp values and will determine the best one, hold 1 to run, press button 2 to end");
    SmartDashboard.putString("Status", "Running Tune Distance Pid");
    oscilationCounter = 0;
    powerCounter = 0;
    turnTimer = 0;
    PowerLevelTimeout = 250;
    secondTurn = false;
    testCompleted = false;
    testKp = _robot.robotMap.minDrivePower/12;
    iterationCounter = 0;
    SmartDashboard.putNumber("Testing Kp Distance", testKp);
    resultsArray = new ArrayList<PidDataDTO>();
    pIncrement = testKp/10;
    bestTime = Double.MAX_VALUE;
    _timer = new Timer();
    loopStart = Timer.getFPGATimestamp();
    loopEnd = Double.MAX_VALUE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_robot.stick.getRawButton(1)){
        double target = 0;
        boolean moving = false;  
        secondTurn = false;
        if(iterationCounter % 2 == 0){
            target = 24;
            secondTurn = true;
        }
        if(turnTimer == 0){
            // _robot.leftMaster.setSelectedSensorPosition(0, 0, 10);
            // _robot.rightMaster.setSelectedSensorPosition(0, 0, 10);
            _distPid = new DistancePid(testKp, 0, 0, _robot.robotMap.minTurnPower, .002, 1, _robot);
            _distPid.SetTargetDistance(target);
            SmartDashboard.putString("Status", "RunningKp: "+ Double.toString(testKp) + " target: " + Double.toString(target) + " test #" + Integer.toString(iterationCounter));
            _timer.reset();
            _timer.start();
            loopStart = Timer.getFPGATimestamp();
            oscilationCounter = 0;
            stoppedCounter = 0;
            SmartDashboard.putNumber("Testing Kp Distance", testKp);
        }
        SmartDashboard.putNumber("pidDist Turn Timer", turnTimer);
        turnTimer++;
        double power = _distPid.GetDistancePidOutput();
        _robot.driveTrain.Move(power, power);
        stoppedCounter ++;
        // compare lastPower to current to check for sign flipped;
        if(lastPower*power < 0){
            oscilationCounter ++;
        }
        lastPower = power;
        if (power == 0){
            stoppedCounter ++;
            SmartDashboard.putBoolean("Detected Stop", true);
            SmartDashboard.putNumber("stopped counter", stoppedCounter);
            if(stoppedCounter == 1){
                //testTime = _timer.getFPGATimestamp();
                testTime = turnTimer*20;
              SmartDashboard.putNumber("pidDistTune test time",testTime);
            }
        }else{
            stoppedCounter = 0;
           // SmartDashboard.putNumber("test time", 0);
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
                
                bestTime = testTime;
                SmartDashboard.putNumber("Tune DistPid Best Time", bestTime);
            }else{
                double timeDif = Math.abs(bestTime - testTime);
                SmartDashboard.putNumber("testTimeDiff", timeDif);
                if(timeDif < 50){
                    testCompleted = true;
                }
            }
            stoppedCounter = 0;
            turnTimer = 0;
        }else{
            SmartDashboard.putBoolean("Detected Stop", false);
        }
        if(oscilationCounter > 5){
            PidDataDTO pidResult = new PidDataDTO();
            pidResult.Kp = testKp;
            pidResult.time = Double.MAX_VALUE;
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
            pidResult.time = Double.MAX_VALUE;
            resultsArray.add(pidResult);
            turnTimer =0;
            iterationCounter ++;
            testKp = testKp + pIncrement;
            
        }
        
    }
    else{
        _robot.driveTrain.Move(0,0);
    }

    
    
  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    boolean done = _robot.stick.getRawButton(2) || testCompleted;
    if(done){
        
        SmartDashboard.putString("Status", "Determined best dist Kp: "+ Double.toString(testKp));
       
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
    _robot.driveTrain.Move(0,0);
  }
}
