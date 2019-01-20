/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.AdjustLift;

/**
 * Add your docs here.
 */
public class PistonLift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid sol = new DoubleSolenoid(RobotMap.doubleSolChan1, RobotMap.doubleSolChan2);

  private boolean solValue = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new AdjustLift(false));
  }

  public void toggleSol(){
    if(!solValue){
      sol.set(Value.kForward);
    }
    else{
      sol.set(Value.kReverse);
    }
    solValue = !solValue;
  }

  public void stopSol(){
    sol.set(Value.kOff);
  }

  public String getSol(){
    if(solValue)
      return "Forward";
    else if(!solValue){
      return "Backward";
    }
    else{
      return "none";
    }
  }
//Ishan looks like a squid
  private static PistonLift instance;
  public static PistonLift getInstance(){
    if(instance == null)
      instance = new PistonLift();
    return instance;  
  }
}
