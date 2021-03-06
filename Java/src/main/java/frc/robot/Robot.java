/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.ExampleAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HatchGrabber;
import frc.robot.subsystems.PistonLift;
import frc.robot.subsystems.vision.HatchVision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot{
  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  Compressor comp = new Compressor();

  /**
   * This function is run when the robot is first started up and is
   * used to initialize the: Operator Interface, Drive Train, and Arm.
   *
   * It also starts the camera captures and compressor compression.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Default Auto", new ExampleAuto());
    chooser.addOption("My Auto", new ExampleAuto());
    SmartDashboard.putData("Auto mode", chooser);

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();
    
    comp.setClosedLoopControl(true);

    DriveTrain.getInstance().setSlow(false)/*.setReversed(false)*/;
    HatchVision.getInstance();
    OI.getInstance();

    HatchGrabber.getInstance().setPowerMode(true);

    Arm.getInstance().resetEncoder(RobotMap.C_ARM_TOP_POS);

    DriveTrain.getInstance().setReversed(false);

    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
  }

  /**
   * This function is called every robot packet, no matter the mode. It is used
   * for diagnostics and other displays on the drivetrain to notify the drivers
   * of any potential issues.
   *
   * Diagnostics include: Voltage, Gyro, Power Mode for the Arm, Drivetrain
   * Display, etc.
   * 
   * It also updates the stored compressor/pressure value.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Gyro", DriveTrain.getInstance().getZHeading()); 
    
    /*SmartDashboard.putData("Drive", DriveTrain.getInstance().getDrive());

    String ballShootState = "OFF";
    if(CargoShooter.getInstance().getBall() > 0)
      ballShootState = "OUT";
    else if(CargoShooter.getInstance().getBall() < 0)
      ballShootState = "IN";
    else
      ballShootState = "OFF";
    
    SmartDashboard.putString("Cargo Shooter", ballShootState);

    SmartDashboard.putString("Front Pistons", (PistonLift.getInstance().getFrontSol())?"ON":"OFF");

    SmartDashboard.putString("Back Pistons", (PistonLift.getInstance().getBackSol())?"ON":"OFF");

    SmartDashboard.putString("Speed", (!DriveTrain.getInstance().getSlow())?"SLOW":"FAST");

    SmartDashboard.putBoolean("Hatch Hold", (HatchGrabber.getInstance().getPowerMode())?true:false);
    
    //SmartDashboard.putNumber("Encoder", Arm.getInstance().getEncoder());

    SmartDashboard.putString("Reversed?", (DriveTrain.getInstance().getReversed())?"REVERSED":"NOT REVERSED");
    
    SmartDashboard.putNumber("P", Arm.getInstance().getP());
    SmartDashboard.putNumber("I", Arm.getInstance().getI());
    SmartDashboard.putNumber("D", Arm.getInstance().getD());
    */

    //SmartDashboard.putNumber("Encoder", Arm.getInstance().getPosition());
    //SmartDashboard.putNumber("Angle", Arm.getInstance().getAngle());

    SmartDashboard.putNumber("Angle", DriveTrain.getInstance().getZHeading());
    //Arm.getInstance().setArmSpeed(.25);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * We calibrate the IMU when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    //DriveTrain.getInstance().calibrateIMU();
  }

  /**
   * This function is called every robot packet when the robot is disabled.
   * We make sure that the scheduler continues running. 
   * 
   */

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. This year, we will
   * not be using autonomous during the robot game. However, it is still practical
   * to have autonomous ready for testing.
   * 
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = chooser.getSelected();

    DriveTrain.getInstance().resetIMU();

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   * We make sure that the scheduler continues running. 
   * 
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // DriveTrain.getInstance().setDeadband(RobotMap.deadbandLimit); //Set deadband on drive controls

    // Make instances of all four attachments: 
    // HatchGrabber, PistonLift, CargoShooter, and Arm.
    HatchGrabber.getInstance(); 
    PistonLift.getInstance();
    CargoShooter.getInstance();
    Arm.getInstance().resetEncoder(RobotMap.C_ARM_TOP_POS);
    
    HatchGrabber.getInstance().setPowerMode(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run(); //Keep running scheduler
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
