

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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


public class Path1 extends Command {
    
    private Robot _robot;
    private int stoppedCounter;
    private boolean testCompleted;
    private DistancePid _distancePid;
    private double _distance;
    private TurnPid _turnPid;
    private double _heading;
    private int trajectoryCounter;
    private Trajectory trajectory;
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftDistanceFollower;
    private DistanceFollower _rightDistanceFollower;

  public Path1(Robot robot, int pathNumber) {
    
    _robot = robot;
    requires(_robot.driveTrain);
    
    stoppedCounter = 0;
    trajectoryCounter = 0;


  }

  @Override
  protected void initialize() {
    Waypoint[] points = new Waypoint[] {
        new Waypoint(.5, .5, Pathfinder.d2r(45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
        new Waypoint(0, 1, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        new Waypoint(0, 1.5, 0)
    };                           // Waypoint @ x=0, y=0,   exit angle=0 radians
       
   // Create the Trajectory Configuration
//
// Arguments:
// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
// Sample Count:        SAMPLES_HIGH (100 000)
//                      SAMPLES_LOW  (10 000)
//                      SAMPLES_FAST (1 000)
//Time Step:           0.05 Seconds
// Max Velocity:        1.7 m/s
// Max Acceleration:    2.0 m/s/s
// Max Jerk:            60.0 m/s/s/s


Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.02, Helpers.FeetToMeters(RobotMap.maxVelocity), Helpers.FeetToMeters(RobotMap.maxAccel), 60.0);
SmartDashboard.putString("Status", "Running Path1 "  );

// Generate the trajectory
 trajectory = Pathfinder.generate(points, config);
    SmartDashboard.putString("Status", "Running Path1: trjectory calculated"  );
    testCompleted = false;
    stoppedCounter = 0;
   
    _turnPid = new TurnPid(RobotMap.kp_Angle, 0, 0, RobotMap.minTurnPower, _robot.getPeriod(), RobotMap.pidTDistDeadband);
    TankModifier mod = new TankModifier(trajectory).modify(Helpers.InchesToMeters(RobotMap.trackWidth));
    _leftTrajectory = mod.getLeftTrajectory();
    _rightTrajectory = mod.getRightTrajectory();
    _leftDistanceFollower = new DistanceFollower(_leftTrajectory);
    _rightDistanceFollower = new DistanceFollower(_rightTrajectory);
    _leftDistanceFollower.configurePIDVA(RobotMap.KpDistanceFollower, 0.0, 0.0, Helpers.FeetToMeters(RobotMap.maxVelocity)/12.5, Helpers.FeetToMeters(RobotMap.maxAccel)/12.5);
    _leftDistanceFollower.reset();
    _rightDistanceFollower.configurePIDVA(RobotMap.KpDistanceFollower, 0.0, 0.0, Helpers.FeetToMeters(RobotMap.maxVelocity)/12.5, Helpers.FeetToMeters(RobotMap.maxAccel)/12.5);
    _rightDistanceFollower.reset();
  }

 
  @Override
  protected void execute() {
      if(_robot.stick.getRawButton(1)){
        double lCurrentDistance = _robot.leftMaster.getSelectedSensorPosition(0)/RobotMap.driveEncoderTicksPerInch;
        double rCurrentDistance = _robot.rightMaster.getSelectedSensorPosition(0)/RobotMap.driveEncoderTicksPerInch;
        double leftPower = _leftDistanceFollower.calculate(Helpers.FeetToMeters(lCurrentDistance));
        double rightPower = _rightDistanceFollower.calculate(Helpers.FeetToMeters(rCurrentDistance));
        double targetHeading = _leftDistanceFollower.getHeading();
        _turnPid.SetTargetAngle(targetHeading);
        double turnPower = _turnPid.GetAnglePidOutput(_robot.navX.getYaw());
        SmartDashboard.putNumber("Pathfollower lCurrentDistance", lCurrentDistance);
        SmartDashboard.putNumber("Pathfollower rCurrentDistance", rCurrentDistance);
        SmartDashboard.putNumber("Pathfollower leftPower", leftPower);
        SmartDashboard.putNumber("Pathfollower rightPower", rightPower);
        SmartDashboard.putNumber("Pathfollower targetHeading", targetHeading);


        _robot.driveTrain.Move(leftPower - turnPower, rightPower + turnPower); 
      }else{
        _robot.driveTrain.Move(0, 0); 
      }
        

        

  }

  
  @Override
  protected boolean isFinished() {
    
    boolean done = _leftDistanceFollower.isFinished();
    if(done || _robot.stick.getRawButton(2)){
        
        SmartDashboard.putString("Status", "Completed driving path 1");
        _robot.driveTrain.Move(0, 0); 
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
