/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class ShiftSlow extends InstantCommand {
  /**
   * Add your docs here.
   */
  boolean value;

  public ShiftSlow(boolean v) {
    super();
    requires(DriveTrain.getInstance());
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    value = v;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    DriveTrain.getInstance().setSlow(value);
  }

}
