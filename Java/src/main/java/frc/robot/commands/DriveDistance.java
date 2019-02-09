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
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends Command {
  private static final double P = 0.21,
								              I = 0,
                              D = 0.05,
                              tolerance = 0.2; /* Tolerance in percentage that we will allow for this action;
												                        * Look here if the next action isn't executing.
                                                */
	
	DriveTrain dt_s = DriveTrain.getInstance();
	private PIDController pid;
	
	public DriveDistance(double inches, double maxSpeed) {
		this(inches);
		pid.setOutputRange(-maxSpeed, maxSpeed);
	}
	
  public DriveDistance(double inches) {
      requires(dt_s);
      
      pid = new PIDController(P, I, D, new PIDSource(){
        
        PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
          m_sourceType = pidSource;
        }
      
        @Override
        public double pidGet() {
          double[] dists = null;
          //dists = dt_s.getEncoderDist();
          return (dists[0] + dists[1])/2;
        }
      
        @Override
        public PIDSourceType getPIDSourceType() {
          return m_sourceType;
        }
      }, d -> dt_s.turnRate(d));
      
      pid.setInputRange(-2 * inches, 2 * inches);
      pid.setOutputRange(-0.5, 0.5);
      pid.setPercentTolerance(tolerance);
      pid.setSetpoint(inches);
      
      setInterruptible(true);
  }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//dt_s.resetEncoders();
    	pid.reset();
    	pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return pid.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	pid.disable();
    	dt_s.driveWheels(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	pid.disable();
    	dt_s.driveWheels(0, 0);
    }
}
