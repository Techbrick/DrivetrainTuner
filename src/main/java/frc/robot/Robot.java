/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
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
  
  public static OI m_oi;
  public RobotMap robotMap = new RobotMap();
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  public 
  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  
  public Joystick stick;
	public  double encoderConstant;
	
  // public TalonSRX leftMaster;
  // private TalonSRX leftFollower;
  // public TalonSRX rightMaster;
  // private TalonSRX rightFollower;
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
    robotMap.verbose = true;
		
		//
		// Configure drivetrain movement
    //
    navX = new AHRS(SPI.Port.kMXP );
    
    driveTrain = new DriveSubsystem(this);
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData("Drive Encoder Cal", new DriveEncoderCal(this));
    SmartDashboard.putData("Manual Drive", new ManualDrive(this));
    SmartDashboard.putData("Min Turn Power", new FindMinTurnPower(this));
    SmartDashboard.putData("Min Drive Power", new FindMinDrivePower(this));
    SmartDashboard.putData("Tune Turn Pid", new TuneTurnPid(this));
    SmartDashboard.putData("Tune Distance Pid", new TuneDistancePid(this));
    SmartDashboard.putData("Turn left 90", new TestTurnLeft90(this));
    SmartDashboard.putData("Test Turn Right 90", new TestTurnRight90(this));
    SmartDashboard.putData("Test Fwd 48", new TestMoveFwd48(this));
    SmartDashboard.putData("Test back 48", new TestMoveBack48(this));

		
    

    
		
    
    m_chooser.addObject("Drive Fwd 24 inches", new DriveDistanceAndDirection(this, 24, 0));
    m_chooser.addObject("Drive dog leg right", new DogLegRight(this));
    m_chooser.addObject("Drive dog leg left", new DogLegLeft(this));
    SmartDashboard.putData("Auto mode", m_chooser);
		
    NetworkTableInstance.getDefault().setUpdateRate(0.020);
    
    m_oi = new OI();
    navX.reset();
    navX.zeroYaw();
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();

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
    driveTrain.Move(0,0);
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
  public void testInit() {
    super.testInit();
    // 
    robotMap.verbose = true;
  }

  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    Logger();
    


  }
  //   this function is needed for commands to read position;

  
  private void Logger(){
    if(robotMap.verbose){
      SmartDashboard.putNumber("l_encoder_pos", Math.round(driveTrain.GetLeftEncoderPosition()));
      // SmartDashboard.putNumber("l_encoder_rate", Math.round(leftEncoderRate.get()));
      SmartDashboard.putNumber("r_encoder_pos", Math.round(driveTrain.GetRightEncoderPosition()));
      // SmartDashboard.putNumber("r_encoder_rate", Math.round(rightEncoderRate.get()));
      SmartDashboard.putNumber("navx pitch", Math.round(navX.getPitch()));
      SmartDashboard.putNumber("navx Heading", navX.getCompassHeading());
      SmartDashboard.putNumber("navx Angle", Math.round(navX.getRawMagX()));
      SmartDashboard.putNumber("avgEncoderRate", driveTrain.GetAverageEncoderRate());
    }
    
    double yaw = navX.getYaw();
    boolean navxAlive = navX.isConnected();
    SmartDashboard.putBoolean("navXConnected", navxAlive);
    SmartDashboard.putNumber("navX yaw", Math.round(yaw));
    
    //SmartDashboard.putBoolean("joystick buttom", stick.getRawButton(1));
    double fps = driveTrain.GetAverageEncoderRate()*12;
    SmartDashboard.putNumber("fps", fps);
    
  }

}
