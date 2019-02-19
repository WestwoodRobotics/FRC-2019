/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.HatchGrabber;
import frc.robot.subsystems.vision.HatchVision;

/**
 * Add your docs here.
 */
public class TurnToHatch extends InstantCommand {
  /**
   * Add your docs here.
   */
  double value;

  public TurnToHatch() {
    super();
    System.out.println("TurnTOHatch start");
    
    requires(HatchGrabber.getInstance());
 
    double degrees = HatchVision.getInstance().getAngleFromHatch();
    SmartDashboard.putNumber("Angle Of Hatch From Robot", degrees);
    
    value = degrees;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(value == 420) return;

    TurnTo turnTo = new TurnTo(value);

    turnTo.execute();
    turnTo.close();
  }
}