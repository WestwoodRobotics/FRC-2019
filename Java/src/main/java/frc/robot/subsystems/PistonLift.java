/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.AdjustLift;

/**
 * Add your docs here.
 */
public class PistonLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Solenoid solFront = new Solenoid(RobotMap.liftSolFrontChan);
  private Solenoid solBack = new Solenoid(RobotMap.liftSolBackChan);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new AdjustLift(RobotMap.LiftMode.OFF));
  }

  public PistonLift(){
    //solFront.set(false);
    //solBack.set(false);
  }

  public void toggleFrontSol(){
    solFront.set(!solFront.get());
    SmartDashboard.putBoolean("Front Solenoid", solFront.get());
  }

  public void toggleBackSol(){
    solBack.set(!solBack.get());
    SmartDashboard.putBoolean("Back Solenoid", solBack.get());
  }

  private static PistonLift instance;
  public static PistonLift getInstance(){
    if(instance == null)
      instance = new PistonLift();
    return instance;  
  }
}
