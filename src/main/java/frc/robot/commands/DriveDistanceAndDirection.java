

package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DistancePid;
import frc.robot.Helpers;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TurnPid;

public class DriveDistanceAndDirection extends Command {
    
    private Robot _robot;
    private int stoppedCounter;
    private boolean testCompleted;
    private DistancePid _distancePid;
    private double _distance;
    private TurnPid _turnPid;
    private double _heading;


  public DriveDistanceAndDirection(Robot robot, double distance, double heading) {
    
    _robot = robot;
    requires(_robot.driveTrain);
    _distance = distance;
    _heading = heading;
    stoppedCounter = 0;
    


  }

  @Override
  protected void initialize() {
    
    
   
    SmartDashboard.putString("Status", "Running move " + Double.toString(_distance) + " inches, heading " + Double.toString(_heading) );
    testCompleted = false;
    stoppedCounter = 0;
    _distancePid = new DistancePid(RobotMap.kdistance, 0, 0, RobotMap.minDrivePower, _robot.getPeriod(), RobotMap.pidTDistDeadband, _robot);
    _distancePid.SetTargetDistance(_distance);
    _turnPid = new TurnPid(RobotMap.kp_Angle, 0, 0, RobotMap.minTurnPower, _robot.getPeriod(), RobotMap.pidTDistDeadband);
    _turnPid.SetTargetAngle(_heading);
  }

 
  @Override
  protected void execute() {
   
        double power = _distancePid.GetDistancePidOutput();
        double turnPower = _turnPid.GetAnglePidOutput(_robot.navX.getYaw());
        _robot.driveTrain.Move(power - turnPower, power + turnPower); 

        if (power + turnPower == 0){
            stoppedCounter ++;
           
        }else{
            stoppedCounter = 0;
            
        }
        if (stoppedCounter > 5){
            testCompleted = true;
        }

  }

  
  @Override
  protected boolean isFinished() {
    
    boolean done = testCompleted;
    if(done){
        
        SmartDashboard.putString("Status", "Completed drive distance and direction");
        return true;
    }
    return false;
  }

  
  @Override
  protected void end() {
    
    _robot.driveTrain.Move(0, 0); 
  }

  
  @Override
  protected void interrupted() {
    
  }
}
