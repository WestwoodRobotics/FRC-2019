/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GrabHatch;

/**
 * Add your docs here.
 */
public class HatchGrabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid hatchSol = new DoubleSolenoid(1, 3); 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new GrabHatch(false));
  }

  public HatchGrabber(){
    hatchSol.set(Value.kReverse);
  }

  public void toggle(){
    if(hatchSol.get().equals(Value.kForward))
      hatchSol.set(Value.kReverse);
    else if(hatchSol.get().equals(Value.kReverse))
      hatchSol.set(Value.kForward);
  }

  public Value getHatch(){
    return hatchSol.get();
  }

  //Provides for one singular operator interface across all files
  private static HatchGrabber instance;
  public static HatchGrabber getInstance() {
    if(instance == null) {
      instance = new HatchGrabber();
    }
    
    return instance;
  }
}
