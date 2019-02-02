/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends Command {
  DriveTrain dt_s = DriveTrain.getInstance();
  private boolean status = false;

  public DriveStraight(Boolean status) {
    this.status = status;
  }

  @Override
  protected void intialize() {
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
    dt_s.setDriveSlowMode(status);
  }

  @Override
  protected void interrupted() {
  }
}
