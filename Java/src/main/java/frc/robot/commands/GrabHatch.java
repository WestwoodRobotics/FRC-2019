/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.HatchGrabber;

/**
 * Add your docs here.
 */
public class GrabHatch extends Command {
  /**
   * Add your docs here.
   */
  double speed = 0;

  HatchGrabber hatch = HatchGrabber.getInstance();

  

  public GrabHatch(double speed) {
    requires(HatchGrabber.getInstance());
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*if(OI.getInstance().getLPOV() == 0 || OI.getInstance().getLogitechRJoyY() < -0.25){
      hatch.set(-.275);
    }
    else if(OI.getInstance().getLPOV() == 180 || OI.getInstance().getLogitechRJoyY() > 0.25){
      hatch.set(.275);
    }
    else*/
      hatch.set(0);
  }
 
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
