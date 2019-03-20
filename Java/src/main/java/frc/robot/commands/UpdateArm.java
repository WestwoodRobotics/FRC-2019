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
import frc.robot.OI;
import frc.robot.subsystems.Arm;

public class UpdateArm extends Command {
  private Arm arm = Arm.getInstance();

  public static final double P = 1,
                             I = 0,
                             D = 0,
                             absoluteTolerance = 10;

  private PIDController pid;

  public UpdateArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(arm);

    /*pid = new PIDController(P, I, D, new PIDSource(){
    
      PIDSourceType sourceType = PIDSourceType.kDisplacement;

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        sourceType = pidSource;
      }
    
      @Override
      public double pidGet() {
        return arm.getEncoder();
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return sourceType;
      }
    }, s -> arm.setArmSpeed(s));

    pid.setInputRange(0, 70000);
    pid.setOutputRange(-0.5, 0.5);
    pid.setAbsoluteTolerance(absoluteTolerance);
    pid.setSetpoint(arm.getAbsolutePos());*/
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pid.reset();
    pid.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*if(OI.getInstance().getLogitechPOV() == 0)
      arm.setArm(1);
    else if(OI.getInstance().getLogitechPOV() == 180)
      arm.setArm(-1);
    */
    //pid.setSetpoint(arm.getAbsolutePos());
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
