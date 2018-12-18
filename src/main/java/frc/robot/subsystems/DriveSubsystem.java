package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX _leftMaster;
  private TalonSRX _leftFollower;
  private TalonSRX _rightMaster;
  private TalonSRX _rightFollower;
  private Robot _robot;
  private double encoderConstant;
  Supplier<Double> leftEncoderPosition;
	Supplier<Double> leftEncoderRate;
	Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;

  // public DriveSubsystem(Robot robot){
  //   _robot = robot;
  //   _leftMasterTalon =robot.leftMaster;
  //   _rightMasterTalon = robot.rightMaster;

  // }
  public DriveSubsystem (Robot robot){
    _leftMaster = new TalonSRX(robot.robotMap.leftMaster);
    _leftFollower = new TalonSRX(robot.robotMap.rightMaster);
    _rightMaster = new TalonSRX(robot.robotMap.leftFollower);
    _rightFollower  = new TalonSRX(robot.robotMap.rightFollower);
    _leftFollower.setInverted(true);
    _leftFollower.follow(_leftMaster);
    _rightFollower.follow(_rightMaster);
    encoderConstant = (1 / _robot.robotMap.driveEncoderTicksPerInch);
    _leftMaster.clearStickyFaults(30);
    _rightMaster.clearStickyFaults(30);
    _leftMaster.setNeutralMode(NeutralMode.Brake);
    _leftMaster.setNeutralMode(NeutralMode.Brake);
    _leftFollower.clearStickyFaults(30);
    _rightFollower.clearStickyFaults(30);
    _leftFollower.setNeutralMode(NeutralMode.Brake);
    _rightFollower.setNeutralMode(NeutralMode.Brake);
    _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 10);
    _leftMaster.setSelectedSensorPosition(0, 0, 10);
    _leftMaster.setSensorPhase(true);
		leftEncoderPosition = () -> _leftMaster.getSelectedSensorPosition(0) * encoderConstant;
		leftEncoderRate = () -> _leftMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
    _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    
    _rightMaster.setSelectedSensorPosition(0, 0, 10);
		rightEncoderPosition = () -> _rightMaster.getSelectedSensorPosition(0) * encoderConstant;
		rightEncoderRate = () -> _rightMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
  }
  public void Move(double leftpower, double rightpower){
    _leftMaster.set(ControlMode.PercentOutput, leftpower);
    _rightMaster.set(ControlMode.PercentOutput, -rightpower);
    SmartDashboard.putString("DriveTrainStatus", "Move power: "+ Double.toString(leftpower) + ", " + Double.toString(rightpower));
  }
  public void ArcadeDrive(double power, double turn){
    double turnPower = turn;
    _leftMaster.set(ControlMode.PercentOutput, power+turnPower);
    _rightMaster.set(ControlMode.PercentOutput, -power+turnPower);
    SmartDashboard.putString("DriveTrainStatus", "ArcadeDrive power: "+ Double.toString(power));
  }
  public double GetLeftEncoderPosition(){
    double left = leftEncoderPosition.get();
    return left;
  }
  public double GetRightEncoderPosition(){
    double right = rightEncoderPosition.get();
    return right;
  }
  public double GetAverageEncoderPosition(){
    double left = leftEncoderPosition.get();
    double right = rightEncoderPosition.get();
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderRate(){
    double left = leftEncoderRate.get();
    double right = rightEncoderRate.get();
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderPositionRaw(){
    double left = _leftMaster.getSelectedSensorPosition(0);
    double right = _rightMaster.getSelectedSensorPosition(0);
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderRateRaw(){
    double left = _leftMaster.getSelectedSensorVelocity(0);
    double right = _rightMaster.getSelectedSensorVelocity(0);
    double result = (left + right)/2;
    return result;

  }
  public void ResetEncoders(){
    _leftMaster.setSelectedSensorPosition(0, 0, 10);
    _rightMaster.setSelectedSensorPosition(0, 0, 10);
  }
  public double GetLeftOutputVoltage(){
    return _leftMaster.getMotorOutputVoltage();

  }
  public double GetRightOutputVoltage(){
    return _rightMaster.getMotorOutputVoltage();

  }
  @Override
  public void initDefaultCommand() {
    
    //setDefaultCommand(new ManualDrive(_robot));
  }
}