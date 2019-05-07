/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
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

  private Spark hatchMotor = new Spark(RobotMap.P_HATCH_GRAB_SPARK);
  private DigitalInput limitSwitch = new DigitalInput(RobotMap.P_HATCH_GRAB_LIMSWITCH);

  boolean powerMode = true;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new GrabHatch(0));
  }

  public HatchGrabber(){
    hatchMotor.stopMotor();
    powerMode = false;
  }

  public void set(double speed){
    if(!limitSwitch.get() && speed > 0)
      hatchMotor.stopMotor();
    else
      hatchMotor.set(speed);
  }

  public double get(){
    return hatchMotor.get();
  }

  public boolean getPowerMode(){
    return powerMode;
  }

  public void setPowerMode(boolean p){
    powerMode = p;
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
