/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.Supplier;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.omg.CORBA.PRIVATE_MEMBER;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  RobotMap _robotMap = new RobotMap();
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  Supplier<Double> leftEncoderPosition;
	Supplier<Double> leftEncoderRate;
	Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  
  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  
  public Joystick stick;
	public  double encoderConstant;
	
  public TalonSRX leftMaster;
  private TalonSRX leftFollower;
  public TalonSRX rightMaster;
  private TalonSRX rightFollower;
  public  DriveSubsystem driveTrain;
  public AHRS navX;

  double priorAutospeed = 0;
	Number[] numberArray = new Number[9];

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
   
    SmartDashboard.putString("Instructions", "");
    SmartDashboard.putString("Status", "");
    stick = new Joystick(0);

		leftMaster = new TalonSRX(RobotMap.leftMaster);
    leftFollower = new TalonSRX(RobotMap.leftFollower);
    rightMaster = new TalonSRX(RobotMap.rightMaster);
    rightFollower  = new TalonSRX(RobotMap.rightFollower);
    leftFollower.setInverted(true);
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    
    leftMaster.clearStickyFaults(30);
    rightMaster.clearStickyFaults(30);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.clearStickyFaults(30);
    rightFollower.clearStickyFaults(30);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
		//
		// Configure drivetrain movement
    //
    navX = new AHRS(SPI.Port.kMXP );
    
    driveTrain = new DriveSubsystem(this);
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData("DriveEncoderCal", new DriveEncoderCal(this));
    SmartDashboard.putData("Manual Drive", new ManualDrive(this));
		
    double encoderConstant = (1 / RobotMap.driveEncoderTicksPerInch);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 10);
    leftMaster.setSelectedSensorPosition(0, 0, 10);
		leftEncoderPosition = () -> leftMaster.getSelectedSensorPosition(0) * encoderConstant;
		leftEncoderRate = () -> leftMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.setSensorPhase(true);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
		rightEncoderPosition = () -> rightMaster.getSelectedSensorPosition(0) * encoderConstant;
		rightEncoderRate = () -> rightMaster.getSelectedSensorVelocity(0) * encoderConstant * 0.1;
		
    m_chooser.addDefault("Default Auto", new ExampleCommand(this));
    m_chooser.addObject("Drive Encoder Cal", new DriveEncoderCal(this));
    SmartDashboard.putData("Auto mode", m_chooser);
		// Set the update rate instead of using flush because of a ntcore bug
		// -> probably don't want to do this on a robot in competition
		NetworkTableInstance.getDefault().setUpdateRate(0.010);
    m_oi = new OI();
    navX.reset();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    Logger();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    Logger();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    double power =  stick.getY();
    double twist = stick.getTwist();
    //driveTrain.ArcadeDrive(power, twist);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  //   this function is needed for commands to read position;

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
    double left = leftMaster.getSelectedSensorPosition(0);
    double right = rightMaster.getSelectedSensorPosition(0);
    double result = (left + right)/2;
    return result;

  }
  public double GetAverageEncoderRateRaw(){
    double left = leftMaster.getSelectedSensorVelocity(0);
    double right = rightMaster.getSelectedSensorVelocity(0);
    double result = (left + right)/2;
    return result;

  }
  private void Logger(){
    SmartDashboard.putNumber("l_encoder_pos", Math.round(leftEncoderPosition.get()));
		SmartDashboard.putNumber("l_encoder_rate", Math.round(leftEncoderRate.get()));
		SmartDashboard.putNumber("r_encoder_pos", Math.round(rightEncoderPosition.get()));
    SmartDashboard.putNumber("r_encoder_rate", Math.round(rightEncoderRate.get()));
    double yaw = navX.getYaw();
    boolean navxAlive = navX.isConnected();
    SmartDashboard.putBoolean("navXConnected", navxAlive);
    SmartDashboard.putNumber("navX yaw", Math.round(yaw));
    SmartDashboard.putNumber("navx pitch", Math.round(navX.getPitch()));
    SmartDashboard.putNumber("navx Heading", navX.getCompassHeading());
    SmartDashboard.putNumber("navx Angle", Math.round(navX.getRawMagX()));
    SmartDashboard.putBoolean("joystick buttom", stick.getRawButton(1));
    double fps = GetAverageEncoderRate()*12;
    SmartDashboard.putNumber("fps", fps);
    SmartDashboard.putNumber("rEnc Raw", rightMaster.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("lEnc Raw", leftMaster.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("raw Enc Avg", GetAverageEncoderRateRaw());
  }

}
