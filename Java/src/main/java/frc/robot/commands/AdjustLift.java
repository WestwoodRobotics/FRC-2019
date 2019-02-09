/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
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
  
  private boolean frontSol = false;
  private boolean backSol = false;

  private RobotMap.LiftMode mode = RobotMap.LiftMode.SET;

  public AdjustLift(RobotMap.LiftMode mode, boolean frontSol, boolean backSol) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(PistonLift.getInstance());

    this.mode = mode;

    if(mode == RobotMap.LiftMode.SET){
      this.frontSol = frontSol;
      this.backSol = backSol;
    }
  }

  public AdjustLift(RobotMap.LiftMode mode){
    this.mode = mode;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(mode == RobotMap.LiftMode.SET){
      pl.setFrontSol(frontSol);
      pl.setBackSol(backSol);
    }
    else if(mode == RobotMap.LiftMode.TOGGLE_FRONT){
      pl.toggleFrontSol();
    }
    else if(mode == RobotMap.LiftMode.TOGGLE_BACK){
      pl.toggleBackSol();
    }
  }

}
