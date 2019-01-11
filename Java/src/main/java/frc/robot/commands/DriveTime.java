/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class DriveTime extends Command {

  private DriveTrain dt_s = DriveTrain.getInstance();

  public DriveTime(double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(dt_s);

    setInterruptible(true);
    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt_s.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dt_s.driveWheels(RobotMap.driveTimeSpeed, RobotMap.driveTimeSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    dt_s.driveWheels(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    dt_s.driveWheels(0, 0);
  }
}
