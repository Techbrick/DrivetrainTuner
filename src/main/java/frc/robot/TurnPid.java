package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnPid{

    private double _kp;
    private double _ki;
    private double _kd;
    private double _interval;
    private double _minTurnPower;
    private double _lastError;
    private double _accumulatedI;
    private boolean start;
    private double _deadband;
    private double _targetAngle;
    private boolean _verbose;
    private Robot _robot;
    private double _maxPidPower;

    public TurnPid(double kp, double ki, double kd, double minTurnPower, double interval, double deadband){
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _minTurnPower = minTurnPower;
        _interval = interval;
        _accumulatedI = 0;
        _deadband = deadband;
        _maxPidPower = .5;
        _verbose = false;
        start = true;
    }
    public TurnPid(Robot robot){
        _robot = robot;
        _kp = _robot.robotMap.kp_Angle;
        _ki = _robot.robotMap.ki_Angle;
        _kd = _robot.robotMap.kd_Angle;
        _minTurnPower = _robot.robotMap.minTurnPower;
        _interval = _robot.robotMap.timingInterval;
        _accumulatedI = 0;
        _deadband = _robot.robotMap.pidTurnDeadband;
        _verbose = _robot.robotMap.verbose;
        _maxPidPower = _robot.robotMap.maxPidPower;
        start = true;
    }

    public void SetTargetAngle(double targetAngle){
        _targetAngle = targetAngle;
    }

    public double GetAnglePidOutput(double currentAngle) {
        currentAngle = Helpers.ConvertYawToHeading(currentAngle);
        SmartDashboard.putNumber("target", _targetAngle);
        SmartDashboard.putNumber("current angle", currentAngle);
        if(start){

            SmartDashboard.putString("Pid t Status", "Started New PidTurn Class");
        }
        double angle_error = _targetAngle - currentAngle ; //calculate error
        if(_verbose){
            SmartDashboard.putNumber("TEST angle error", angle_error);
        }
        if(angle_error > 180){
            angle_error = 360-angle_error;
        }else if(angle_error < -180){
            angle_error = angle_error + 360;
        }
        //angle_error = Math.abs(angle_error) > 180 ? 180 - angle_error : angle_error; //scale error to take shortest path
        // if (_targetAngle == 0 && currentAngle > 180) {
        //         angle_error = currentAngle - 360;
        // }
        if(_verbose){
            SmartDashboard.putNumber("TEST angle error corr", angle_error);
        }
        double p_Angle = _kp * angle_error; //calculate p
        _accumulatedI += _ki * (angle_error * _interval); //calculate i
        double i_Angle = _ki*_accumulatedI;
        double d_Angle = 0;
        if (!start){
            d_Angle = _kd * ((angle_error - _lastError) / _interval); //calculate d
        }
        start = false;
        
        double angleOutput = p_Angle + i_Angle + d_Angle; //calculate output
        _lastError = angle_error; //set last angle error for d value
        if(_verbose){
            SmartDashboard.putNumber("TEST angle pwr Raw", angleOutput);
        }
      
      
        angleOutput = Math.abs(angleOutput) < _minTurnPower ? Math.copySign(_minTurnPower, angleOutput) : angleOutput; //if angleOutput is below min, set to min
        angleOutput = Math.abs(angleOutput) > _maxPidPower ? Math.copySign(_maxPidPower, angleOutput) : angleOutput; //if angleOutput is above max, set to max
        //angleOutput = angle_error < 0 ? angleOutput : -angleOutput;
        if (Math.abs(angle_error) < _deadband) { //if done moving
            i_Angle = 0;
            angleOutput = 0;
            SmartDashboard.putString("Pid t Status", "PidTurn Class completed");
        }
        //angleOutput = -angleOutput;
        if(_verbose){
            SmartDashboard.putNumber("TEST angle pwr ", angleOutput);
        }
      
        return -angleOutput;
      }




}