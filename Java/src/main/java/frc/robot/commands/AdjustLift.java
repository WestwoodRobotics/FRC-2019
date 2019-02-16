/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.PistonLift;

/**
 * Add your docs here.
 */
public class AdjustLift extends InstantCommand {
  /**
   * Add your docs here.
   */
  public PistonLift pl = PistonLift.getInstance();

  private RobotMap.LiftMode mode;

  public AdjustLift(RobotMap.LiftMode mode) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(PistonLift.getInstance());

    this.mode = mode;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(mode.equals(RobotMap.LiftMode.TOGGLE_FRONT)){
      pl.toggleFrontSol();
    }
    else if(mode.equals(RobotMap.LiftMode.TOGGLE_BACK)){
      pl.toggleBackSol();
    }
  }

}
