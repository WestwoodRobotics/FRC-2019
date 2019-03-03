/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

public class TankDrive extends Command {

  OI m_oi = OI.getInstance();
  DriveTrain dt_s = DriveTrain.getInstance();

  private double rightSpd;
  private double leftSpd;

  public TankDrive() {
    requires(dt_s);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt_s.setSquaredInputs(true); //Provides finer control at lower inputs of joystick by squaring value and reapplying sign
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double lJoyVal = OI.getInstance().getLJoyY();
    double rJoyVal = OI.getInstance().getRJoyY();

    if(dt_s.getSlow()){
      leftSpd = Math.round(lJoyVal);
      rightSpd = Math.round(rJoyVal);
    }
    else if(!dt_s.getSlow()){
      leftSpd = Math.round(lJoyVal)*.5;
      rightSpd = Math.round(rJoyVal)*.5;
    }
    
    dt_s.driveWheels(leftSpd, rightSpd);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    dt_s.stopWheels();
    dt_s.setSquaredInputs(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
