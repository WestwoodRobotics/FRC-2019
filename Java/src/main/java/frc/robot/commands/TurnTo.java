/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class TurnTo extends Command {
  
  public DriveTrain s_dt = DriveTrain.getInstance();

  double degrees;

  public TurnTo(double degrees){
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(s_dt);
    this.degrees = degrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    s_dt.resetIMU();
    s_dt.turnSetpoint(degrees);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return s_dt.turnOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    s_dt.turnDisable();
    s_dt.stopWheels();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    s_dt.turnDisable();
    s_dt.stopWheels();
  }
}
