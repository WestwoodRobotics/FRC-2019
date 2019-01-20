/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean slowMode = false;

  private Spark leftMaster = new Spark(RobotMap.leftSpark1Port),
                       rightMaster = new Spark(RobotMap.rightSpark1Port);

  private WPI_VictorSPX left2 = new WPI_VictorSPX(RobotMap.leftVictor2Port),
                        right2 = new WPI_VictorSPX(RobotMap.rightVictor2Port);

  private SpeedControllerGroup gR = new SpeedControllerGroup(rightMaster, right2);
  private SpeedControllerGroup gL = new SpeedControllerGroup(leftMaster, left2);

  private DifferentialDrive drive = new DifferentialDrive(gR, gL);

  private Encoder rightEnc = new Encoder(RobotMap.rightEncA, RobotMap.rightEncB, false, Encoder.EncodingType.k1X);
  private Encoder leftEnc = new Encoder(RobotMap.leftEncA, RobotMap.leftEncB, false, Encoder.EncodingType.k1X);

  private boolean squaredInputs = false; //Provides finer control at lower inputs of joystick by squaring value and reapplying sign

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public DriveTrain(){
    left2.set(ControlMode.Follower, RobotMap.leftSpark1Port);
    right2.set(ControlMode.Follower, RobotMap.rightSpark1Port);

    //leftMaster.configOpenloopRamp(ramp, 25);
    //left2.configOpenloopRamp(ramp, 25);
    
    //rightMaster.configOpenloopRamp(ramp, 25);
    //right2.configOpenloopRamp(ramp, 25);

    rightEnc.reset();
    leftEnc.reset();
    
    //setDeadband(0.05);
    this.calibrateGyro();
    this.resetGyro();
  }

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

  public void setSlow(boolean isSlowMode){
    slowMode = isSlowMode;
  }

  public boolean getSlow(){
    return slowMode;
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

  public double getAngle(){
    return gyro.getAngle();
  }

  public boolean isGyroConnected(){
    return gyro.isConnected();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  public void resetGyro(){
    gyro.reset();
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
