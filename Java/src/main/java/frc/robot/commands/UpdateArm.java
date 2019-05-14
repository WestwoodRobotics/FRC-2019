/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class UpdateArm extends Command {
  
  Arm s_arm = Arm.getInstance();

  public UpdateArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(s_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    s_arm.setPercentTolerance(RobotMap.C_ARM_PERCENT_TOLERANCE);
    s_arm.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    s_arm.updatePID();
    if(s_arm.onTarget()){
      s_arm.setArmSpeed(s_arm.getCurrOutput());
      s_arm.disable();
      s_arm.resetEncoder((int)s_arm.getSetpoint());
    }
    else{
      s_arm.enable();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    s_arm.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    s_arm.disable();
  }
}
