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
  
  public DriveTrain dt_s = DriveTrain.getInstance();

  private double degrees;

  /*public static final double P = 1.5,
                             I = 0,
                             D = 2.9,
                             absoluteTolerance = 0.6;
    
  private PIDController pid;
  */
  public TurnTo(double degrees) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(dt_s);
    this.degrees = degrees;
    /*pid = new PIDController(P, I, D, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_sourceType = pidSource;
      }
    
      @Override
      public double pidGet() {
        return dt_s.getZHeading();
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return m_sourceType;
      }
    }, d -> dt_s.turnRate(d));

    pid.setInputRange(-720, 720);
    pid.setOutputRange(-0.5, 0.5);
    pid.setAbsoluteTolerance(absoluteTolerance);
    pid.setSetpoint(degrees);*/
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt_s.resetIMU();		// reset gyros
    /*pid.reset();
    pid.enable();*/
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(degrees > 0)
      dt_s.driveWheels(.2, -.2);
    else
      dt_s.driveWheels(-.2, .2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println(degrees - dt_s.getZHeading());
    return Math.abs(degrees - dt_s.getZHeading()) < 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    dt_s.stopWheels();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    dt_s.stopWheels();
  }
}
