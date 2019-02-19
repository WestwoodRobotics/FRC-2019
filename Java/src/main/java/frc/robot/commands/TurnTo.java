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

  /*public static double P = 4,
                       I = 3,
                       D = 16,
                       absoluteTolerance = 0.1;
    
  private PIDController pid;*/

  public TurnTo(double degrees) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(dt_s);
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
    pid.setOutputRange(-0.2, 0.2);
    pid.setAbsoluteTolerance(absoluteTolerance);
    pid.setSetpoint(degrees);*/
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dt_s.resetIMU();		// reset gyros
    dt_s.resetIMU();		// reset gyros
    dt_s.resetIMU();		// reset gyros
    /*pid.reset();

    P = SmartDashboard.getNumber("P", P);
    I = SmartDashboard.getNumber("I", I);
    D = SmartDashboard.getNumber("D", D);
    
    System.out.println(P);
    System.out.println(I);
    System.out.println(D);
    
    pid.enable();*/
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    dt_s.driveWheels(.1, -.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(90 - dt_s.getZHeading()) <= 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //pid.disable();
    dt_s.stopWheels();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    dt_s.stopWheels();
  }
}
