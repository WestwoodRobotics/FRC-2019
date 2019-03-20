/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

/**
 * Add your docs here.
 */
public class SetArmPos extends InstantCommand {
  /**
   * Add your docs here.
   */
  Arm arm = Arm.getInstance();
  RobotMap.ArmEnum position = RobotMap.ArmEnum.TOP;
  int increment = 0;
  public SetArmPos(RobotMap.ArmEnum position) {
    super();
    requires(arm);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = position;
    increment = 0;
  }

  public SetArmPos(int increment){
    this.increment = increment;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(increment == 0)
      arm.setArm(position);
    else
      arm.setArm(increment);
  }

}
