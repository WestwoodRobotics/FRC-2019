/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Arm;

public class AdjustArm extends Command {
  public Arm arm = Arm.getInstance();

  double pos = 0;

  public AdjustArm(double pos) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(arm);
    this.pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    /*arm.setP(SmartDashboard.getNumber("P", arm.getP()));
    arm.setI(SmartDashboard.getNumber("I", arm.getI()));
    arm.setD(SmartDashboard.getNumber("D", arm.getD()));*/

    arm.setPercentTolerance(0.01);
    arm.setSetpoint(pos);
    arm.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    arm.updatePID();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    arm.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    arm.disable();
  }
}
