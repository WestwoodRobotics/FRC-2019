/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
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
  Command m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  Compressor comp = new Compressor();

  /**
   * This function is run when the robot is first started up and is
   * used to initialize the: Operator Interface, Drive Train, and Arm.
   *
   * It also starts the camera captures and compressor compression.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", new ExampleAuto());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);

    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();
    
    comp.setClosedLoopControl(true);

    DriveTrain.getInstance();
    HatchVision.getInstance();
    OI.getInstance();
    Arm.getInstance().setPowerMode(false);
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
    SmartDashboard.putNumber("Gyro", DriveTrain.getInstance().getZHeading()); 

    SmartDashboard.putBoolean("Power Lift", Arm.getInstance().getPowerMode());
    
    SmartDashboard.putData("Drive", DriveTrain.getInstance().getDrive());

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

    SmartDashboard.putString("Hatch Grabber", (HatchGrabber.getInstance().getHatch().equals(Value.kForward))?"OPEN":"CLOSED");

    SmartDashboard.putString("Speed", (DriveTrain.getInstance().getSlow())?"SLOW":"FAST");
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * We calibrate the IMU when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    DriveTrain.getInstance().calibrateIMU();
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
    //m_autonomousCommand = m_chooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // DriveTrain.getInstance().setDeadband(RobotMap.deadbandLimit); //Set deadband on drive controls

    // Make instances of all four attachments: 
    // HatchGrabber, PistonLift, CargoShooter, and Arm.
    HatchGrabber.getInstance(); 
    PistonLift.getInstance();
    CargoShooter.getInstance();
    Arm.getInstance();
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
