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

  public static final double deadbandLimit = 0.499; 

  public static final double driveTimeSpeed = 0.4;

  public static final int liftSolBackChan = 0,
                          hatchGrabberSolPort = 1,
                          liftSolFrontChan = 2;
                          
                          
  public static final int leftTalon1Port = 0,
                          leftTalon2Port = 1,
                          rightTalon1Port = 2,
                          rightTalon2Port = 3;

  public static final int armCameraPort = 0,
                          chassisCameraPort = 1;
                          
  public static final int armTalon1Port = 4,
                          armTalon2Port = 5;

  public static final int encoderForwardLimit = 82135,  //CHANGE THIS
                          encoderBackwardLimit = 2789;

  public static final int cargoShooterTalonPort = 6;

  public enum Cargo{
    OFF,
    OUT,
    IN;
  }

  public enum LiftMode{
    OFF,
    TOGGLE_FRONT,
    TOGGLE_BACK;
  }

  public enum Arm{
    UP,
    DOWN,
    OFF;
  }

}
