/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(RobotMap.leftTalonPort);
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(RobotMap.rightTalonPort);

  private DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  private Encoder rightEnc = new Encoder(RobotMap.rightEncA, RobotMap.rightEncB, false, Encoder.EncodingType.k1X);
  private Encoder leftEnc = new Encoder(RobotMap.leftEncA, RobotMap.leftEncB, false, Encoder.EncodingType.k1X);

  private boolean squaredInputs = false; //Provides finer control at lower inputs of joystick by squaring value and reapplying sign

  @Override
  public void initDefaultCommand(){
    setDefaultCommand(new TankDrive());
  }

  public void driveWheels(double leftSpd, double rightSpd){
    drive.tankDrive(leftSpd, rightSpd, this.squaredInputs);
  }

  public void turnRate(double rt){
    drive.curvatureDrive(0, rt, true);
  }

  public void stopWheels(){
    this.driveWheels(0, 0);
  }

  public void setSquaredInputs(boolean squaredInputs){
    this.squaredInputs = squaredInputs;
  }

  public void setDeadband(double band) {
		drive.setDeadband(band);
  }
  
  //Provides raw encoder value NOT IN INCHES
  public double[] getEncoders(){
    double[] enc = {rightEnc.get(), leftEnc.get()};
    return enc;
  }

  //Provides encoder value IN INCHES
  public double[] getEncoderDist(){
    double[] enc = {rightEnc.getDistance(), leftEnc.getDistance()};
    return enc;
  }

  public void resetEncoders(){
    rightEnc.reset();
    leftEnc.reset();
  }

  public double getZHeading(){
    return 0.0;
  }

  public void resetIMU(){
    
  }

  //Provides for one singular drivetrain across all files
  private static DriveTrain instance;
  public static DriveTrain getInstance() {
    if(instance == null) {
      instance = new DriveTrain();
    }
    
    return instance;
  }
}
