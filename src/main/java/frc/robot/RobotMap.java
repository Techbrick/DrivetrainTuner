/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int driveStick = 0;
  public static final int opStick = 1;

  public static final int leftMaster = 12;
  public static final int leftFollower = 13;
  public static final int rightMaster = 14;
  public static final int rightFollower = 15;
  public static boolean twoSpeedDrive = false;

  private static  double kAngleSetpoint = 0.0;
  private static  double kP = 0.005; // propotional turning constant
  public static  double joystickDeadband = .05;
  public static double driveEncoderTicksPerInch = 2048/6/3.14;
  public static double pidTurnDeadband = 2;

  
}
