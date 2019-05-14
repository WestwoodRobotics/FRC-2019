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
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static final double C_DRIVE_TIME_SPEED = 0.4;

  public static final int P_LIFT_BACK_SOL = 0,
                          P_LIFT_FRONT_SOL = 2;

  public static final int P_HATCH_GRAB_SPARK = 0,
                          P_HATCH_GRAB_LIMSWITCH = 0;
                          
  public static final int P_DRIVE_LEFT_TALON_1 = 0,
                          P_DRIVE_LEFT_TALON_2 = 1,
                          P_DRIVE_RIGHT_TALON_1 = 2,
                          P_DRIVE_RIGHT_TALON_2 = 3;

  public static final int P_ARM_CAM = 0,
                          P_DRIVE_CAM = 1;
                          
  public static final int P_ARM_TALON_1 = 4,
                          P_ARM_TALON_2 = 5;

  public static final int C_TICKS_PER_REV = 4096 * 45;
  public static final double C_HORIZONTAL_VOLTAGE = 0.3;
  public static final double C_ARM_PERCENT_TOLERANCE = 0.15;

  public static final int C_ARM_TOP_POS = -52765,  //CHANGE THIS
                          C_ARM_ROCKET_POS = -6994,
                          C_ARM_SHIP_POS = -48095,
                          C_ARM_BOTTOM_POS = 9074;

  public static final int P_CARGO_SHOOTER_TALON = 6;

  public enum E_CARGO_MOTION{
    OFF,
    OUT,
    IN;
  }

  public enum E_LIFT_MODE{
    OFF,
    TOGGLE_FRONT,
    TOGGLE_BACK;
  }

  public enum E_ARM_POS{
    TOP,
    ROCKET,
    SHIP,
    BOTTOM;
  }

}
