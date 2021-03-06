package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistancePid{

    private double _kp;
    private double _ki;
    private double _kd;
    private double _interval;
    private double _minDrivePower;
    private double _lastError;
    private double _accumulatedI;
    private boolean start;
    private double _deadband;
    private double _targetDistance;
    private Robot _robot;
    private boolean _verbose;
    private double _maxPidPower;

    

    public DistancePid(double kp, double ki, double kd, double minDrivePower, double interval, double deadband, Robot robot){
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _minDrivePower = minDrivePower;
        _interval = interval;
        _accumulatedI = 0;
        _deadband = deadband;
        start = true;
        _robot = robot;
        _verbose = false;
        _maxPidPower = robot.robotMap.maxPidPower;
    }
    public DistancePid(Robot robot){
        _robot = robot;
        _kp = _robot.robotMap.kp_distance;
        _ki = _robot.robotMap.ki_distance;
        _kd = _robot.robotMap.kd_distance;
        _minDrivePower = _robot.robotMap.minDrivePower;
        _interval = _robot.robotMap.timingInterval;
        _accumulatedI = 0;
        _deadband = _robot.robotMap.pidDistDeadband;
        start = true;
        _verbose = _robot.robotMap.verbose;
        _maxPidPower = robot.robotMap.maxPidPower;
    }

    public void SetTargetDistance(double targetDistance){
        //  everything is calculated in inches relative to the position when the method is called.
        _targetDistance = targetDistance + _robot.driveTrain.GetAverageEncoderPosition();
    }

    public double GetDistancePidOutput() {

        if(start){

            SmartDashboard.putString("Pid D Status", "Started New PidDistance Class");
        }
        double distance_error = _targetDistance - _robot.driveTrain.GetAverageEncoderPosition(); //calculate error
        if(_verbose){
            SmartDashboard.putNumber("TEST target dist", _targetDistance);
            SmartDashboard.putNumber("TEST dist error", distance_error);
            SmartDashboard.putNumber("test Current Position", _robot.driveTrain.GetAverageEncoderPosition());
        }
        double p_Distance = _kp * distance_error; //calculate p
        _accumulatedI += _ki * (distance_error * _interval); //calculate i
        double i_Distance = _ki*_accumulatedI;
        double d_Distance = 0;
        if (!start){
            d_Distance = _kd * ((distance_error - _lastError) / _interval); //calculate d
        }
        start = false;
        
        double distancePowerOutput = p_Distance + i_Distance + d_Distance; //calculate output
        _lastError = distance_error; //set last angle error for d value
      
        if(_verbose){
            SmartDashboard.putNumber("TEST DIST pwr Raw", distancePowerOutput);
        }
      
        distancePowerOutput = Math.abs(distancePowerOutput) < _minDrivePower ? Math.copySign(_minDrivePower, distancePowerOutput) : distancePowerOutput; //if distancePowerOutput is below min, set to min
        distancePowerOutput = Math.abs(distancePowerOutput) > _maxPidPower ? Math.copySign(_maxPidPower, distancePowerOutput) : distancePowerOutput; //if distancePowerOutput is above max, set to max
        
        if (Math.abs(distance_error) < _deadband) { //if done moving
            i_Distance = 0;
            distancePowerOutput = 0;
            SmartDashboard.putString("Pid D Status", "ended PidDistance Class");
        }
        if(_verbose){
            SmartDashboard.putNumber("TEST distance pwr ", distancePowerOutput);
        }
        return distancePowerOutput;
      }




}