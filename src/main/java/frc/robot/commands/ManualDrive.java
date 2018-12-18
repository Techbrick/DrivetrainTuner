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
import frc.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class ManualDrive extends Command {
  private Robot _robot; 
  private double _lastLeftVel;
  private double _lastRightVel;
  private double _lastAvgVel;
  private double maxAccell = 0;
  double maxVel = 0;
  private int avgVelCounter = 0;
  private int avgAccellCounter = 0;
  private double avgVelPV = 0;
  private double avgAccellPV = 0;

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
    double power = Helpers.DeadbandJoystick(-_robot.stick.getY(), _robot.robotMap);
    double twist =Helpers.DeadbandJoystick( _robot.stick.getTwist(), _robot.robotMap);
    _robot.driveTrain.ArcadeDrive(power, twist);   
    if(_robot.robotMap.verbose){
        
        double leftDrivePower = _robot.driveTrain.GetLeftOutputVoltage();
        double rightDrivePower = _robot.driveTrain.GetRightOutputVoltage();
        
        double avgInput = (leftDrivePower + rightDrivePower)/2.0;
        SmartDashboard.putNumber("average input power", avgInput);
        double absAvgInput = Math.abs(avgInput);
        double absVel = Math.abs(_robot.driveTrain.GetAverageEncoderRate()*12.0);

        if(absAvgInput > _robot.robotMap.minDrivePower){
            double accel = (absVel - _lastAvgVel)/.02;
            SmartDashboard.putNumber("Accell", accel);
            if(accel > maxAccell ){
                maxAccell = accel;
                SmartDashboard.putNumber("Max FPS/S", maxAccell);
            }
            if(absVel > maxVel){
                maxVel = absVel;
                SmartDashboard.putNumber("Max FPS", maxVel);
            }
            double newAvg = ( avgVelPV * avgVelCounter + absVel/absAvgInput)/(double)(avgVelCounter + 1);
                avgVelCounter ++;
                _robot.robotMap.fpsPerVolt = newAvg;
                SmartDashboard.putNumber("Avg FPS/V", newAvg);
            if(accel > 0){
                

                double newAvgA = (avgAccellPV * avgAccellCounter + accel/avgInput)/(double)(_robot.robotMap.averageCounterAccel + 1);
                avgAccellCounter ++;
                _robot.robotMap.accelPerVolt = newAvgA;
                SmartDashboard.putNumber("Max FPS/V/s", newAvgA);
            }
            
        }
        _lastAvgVel = absVel;
        
    } 

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
