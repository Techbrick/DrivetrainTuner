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
    double power = Helpers.DeadbandJoystick(-_robot.stick.getY());
    double twist =Helpers.DeadbandJoystick( _robot.stick.getTwist()/2);
    _robot.driveTrain.ArcadeDrive(power, twist);   
    if(RobotMap.verbose){
        
        double leftDrivePower = _robot.leftMaster.getMotorOutputVoltage();
        double rightDrivePower = _robot.leftMaster.getMotorOutputVoltage();
        
        double avgInput = (leftDrivePower + rightDrivePower)/2.0;
        double absAvgInput = Math.abs(avgInput);
        double absVel = Math.abs(_robot.GetAverageEncoderRate()*12.0);

        if(absVel > 0.1 && avgInput > RobotMap.minDrivePower){
            double accel = (absVel - _lastAvgVel)/.02;
            if(accel > maxAccell ){
                maxAccell = accel;
                SmartDashboard.putNumber("Max FPS/S", maxAccell);
            }
            if(absVel > maxVel){
                maxVel = absVel;
                SmartDashboard.putNumber("Max FPS", maxVel);
            }
            if(accel < .05* maxAccell){
                double newAvg = ( avgVelPV * avgVelCounter + absVel/absAvgInput)/(double)(avgVelCounter + 1);
                avgVelCounter ++;
                RobotMap.fpsPerVolt = newAvg;
                SmartDashboard.putNumber("Avg FPS/V", newAvg);
            }
            else{
                double newAvg = (avgAccellPV * avgAccellCounter + accel/avgInput)/(double)(RobotMap.averageCounterAccel + 1);
                avgAccellCounter ++;
                RobotMap.accelPerVolt = newAvg;
                SmartDashboard.putNumber("Max FPS/V/s", newAvg);
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
