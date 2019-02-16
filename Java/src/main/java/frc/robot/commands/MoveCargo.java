/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoShooter;

public class MoveCargo extends Command {
  
  RobotMap.Cargo c = RobotMap.Cargo.OFF;
  CargoShooter cs = CargoShooter.getInstance();

  public MoveCargo(RobotMap.Cargo c) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(cs);
    this.c = c;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(c == RobotMap.Cargo.IN || OI.getInstance().getLogitechLJoyY() < -0.2){
      cs.pullBall();
    }
    else if(c == RobotMap.Cargo.OUT || OI.getInstance().getLogitechLJoyY() > 0.2){
      cs.pushBall();
    }
    else{
      cs.stopBall();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //if(c == RobotMap.Cargo.OFF)
      //return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    cs.stopBall();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    cs.stopBall();
  }
}
