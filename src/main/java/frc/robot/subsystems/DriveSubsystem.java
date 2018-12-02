package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.ManualDrive;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX _leftMasterTalon;
  public TalonSRX _rightMasterTalon;
  private Robot _robot;


  public DriveSubsystem(Robot robot){
    _robot = robot;
    _leftMasterTalon =robot.leftMaster;
    _rightMasterTalon = robot.rightMaster;

  }
  public void Move(double leftpower, double rightpower){
    _leftMasterTalon.set(ControlMode.PercentOutput, leftpower);
    _rightMasterTalon.set(ControlMode.PercentOutput, -rightpower);
    SmartDashboard.putString("DriveTrainStatus", "Move power: "+ Double.toString(leftpower));
  }
  public void ArcadeDrive(double power, double turn){
    double turnPower = turn/2;
    _leftMasterTalon.set(ControlMode.PercentOutput, power+turnPower);
    _rightMasterTalon.set(ControlMode.PercentOutput, -power+turnPower);
    SmartDashboard.putString("DriveTrainStatus", "ArcadeDrive power: "+ Double.toString(power));
  }

  @Override
  public void initDefaultCommand() {
    
    //setDefaultCommand(new ManualDrive(_robot));
  }
}