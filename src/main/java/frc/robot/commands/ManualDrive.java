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
    if(RobotMap.verbose){
        
        double leftDrivePower = _robot.leftMaster.getMotorOutputVoltage();
        double rightDrivePower = _robot.leftMaster.getMotorOutputVoltage();
        double avgInput = (leftDrivePower + rightDrivePower/2);
        if(_robot.GetAverageEncoderRate() > 1){
            double accel = _robot.GetAverageEncoderRate()*12/.02;
            if(accel > RobotMap.maxAccel){
                RobotMap.maxAccel = accel;
                SmartDashboard.putNumber("Max FPS/S", RobotMap.maxAccel);
            }
            if(_robot.GetAverageEncoderRate()*12 > RobotMap.maxVelocity){
                RobotMap.maxVelocity = _robot.GetAverageEncoderRate()*12;
                SmartDashboard.putNumber("Max FPS", RobotMap.maxVelocity);
            }
            if(accel < .05* RobotMap.maxAccel){
                double newAvg = (RobotMap.fpsPerVolt * RobotMap.averageCounterVel + _robot.GetAverageEncoderRate()*12/avgInput)/(RobotMap.averageCounterVel + 1);
                RobotMap.averageCounterVel ++;
                RobotMap.fpsPerVolt = newAvg;
                SmartDashboard.putNumber("Avg FPS", RobotMap.fpsPerVolt);
            }
            else{
                double newAvg = (RobotMap.accelPerVolt * RobotMap.averageCounterAccel + accel)/(RobotMap.averageCounterAccel + 1);
                RobotMap.averageCounterAccel ++;
                RobotMap.accelPerVolt = newAvg;
                SmartDashboard.putNumber("Max FPS/s", RobotMap.accelPerVolt);
            }
        }
        


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
