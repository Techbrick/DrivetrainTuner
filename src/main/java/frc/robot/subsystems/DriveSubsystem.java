package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX _leftMasterTalon;
  public TalonSRX _rightMasterTalon;


  public DriveSubsystem(TalonSRX leftMaster, TalonSRX rightMaster){
    _leftMasterTalon =leftMaster;
    _rightMasterTalon = rightMaster;

  }
  public void Move(double leftpower, double rightpower){
    _leftMasterTalon.set(ControlMode.PercentOutput, leftpower);
    _rightMasterTalon.set(ControlMode.PercentOutput, rightpower);
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}