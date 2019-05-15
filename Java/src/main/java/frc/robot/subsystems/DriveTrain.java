/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final double ramp = 0.1;  // acceleration of drive motors in sec

  private boolean slowMode = false;

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.P_DRIVE_LEFT_TALON_1),
                        rightMaster = new WPI_TalonSRX(RobotMap.P_DRIVE_RIGHT_TALON_1);

  private WPI_TalonSRX left2 = new WPI_TalonSRX(RobotMap.P_DRIVE_LEFT_TALON_2),
                        right2 = new WPI_TalonSRX(RobotMap.P_DRIVE_RIGHT_TALON_2);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  //private Encoder rightEnc = new Encoder(RobotMap.rightEncA, RobotMap.rightEncB, false, Encoder.EncodingType.k1X);
  //private Encoder leftEnc = new Encoder(RobotMap.leftEncA, RobotMap.leftEncB, false, Encoder.EncodingType.k1X);

  private boolean reversed = false;

  private boolean squaredInputs = false; //Provides finer control at lower inputs of joystick by squaring value and reapplying sign

  private ADIS16448_IMU imu = new ADIS16448_IMU();

  private double turnP = 0,
                 turnI = 0,
                 turnD = 0;

  private PIDController turnPID;

  public DriveTrain(){
    //Sets the second motor for both the left and right side to follow the first motor
    left2.set(ControlMode.Follower, RobotMap.P_DRIVE_LEFT_TALON_1);
    right2.set(ControlMode.Follower, RobotMap.P_DRIVE_RIGHT_TALON_1);

    //Adds a ramp up (acceleration) for the left side
    leftMaster.configOpenloopRamp(ramp, 25);
    left2.configOpenloopRamp(ramp, 25);
    
    //Adds a ramp up (acceleration) for the right side
    rightMaster.configOpenloopRamp(ramp, 25);
    right2.configOpenloopRamp(ramp, 25);

    //Reset both encoders
    //rightEnc.reset();
    //leftEnc.reset();
    
    reversed = false;
    slowMode = false;

    //Set the deadband
    setDeadband(0.05);

    turnPID = new PIDController(turnP, turnI, turnD, new PIDSource(){
      PIDSourceType sourceType = PIDSourceType.kDisplacement;
      
      @Override
      public double pidGet(){
        return getZHeading();
      }

      @Override
      public void setPIDSourceType(PIDSourceType pidSource){
        sourceType = pidSource;
      }

      @Override
      public PIDSourceType getPIDSourceType(){
        return sourceType;
      }

    }, d -> this.turnRate(d));
    
    turnPID.setInputRange(-720, 720);
    turnPID.setOutputRange(-0.5, 0.5);
    turnPID.setAbsoluteTolerance(0.1);

    //Calibrate and reset the IMU
    imu.reset();
    //imu.calibrate();
  }

  //Default command when the drivetrain is created
  @Override
  public void initDefaultCommand(){
    setDefaultCommand(new TankDrive());
  }

  //Drive the wheels in teleop and auto
  public void driveWheels(double leftSpd, double rightSpd){
    if(leftSpd > 0)
      leftSpd *= 0.95;
    if(rightSpd < 0)
      rightSpd *= 0.95;
    drive.tankDrive(leftSpd, rightSpd, this.squaredInputs);
  }

  public void turnSetpoint(double degrees){
    turnPID.reset();
    turnPID.setSetpoint(degrees);
    turnPID.enable();
  }

  public boolean turnOnTarget(){
    turnP = SmartDashboard.getNumber("P", 0);
    turnI = SmartDashboard.getNumber("I", 0);
    turnD = SmartDashboard.getNumber("D", 0);

    turnPID.setPID(turnP, turnI, turnD);
    return turnPID.onTarget();
  }

  public void turnDisable(){
    turnPID.disable();
  }

  public void setReversed(boolean reversed){
    this.reversed = reversed;
  }

  public boolean getReversed(){
    return reversed;
  }

  //Turn in auto
  public void turnRate(double rt){
    drive.curvatureDrive(0, rt, true);
  }

  //Stop the wheels
  public void stopWheels(){
    this.driveWheels(0, 0);
  }

  //Set the slow mode for the robot (0.5 speed)
  public void setSlow(boolean isSlowMode){
    slowMode = isSlowMode;
  }

  //Get whether the slow mode is enabled or not
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
  /*public double[] getEncoders(){
    double[] enc = {rightEnc.get(), leftEnc.get()};
    return enc;
  }

  //Provides encoder value IN INCHES
  public double[] getEncoderDist(){
    double[] enc = {rightEnc.getDistance(), leftEnc.getDistance()};
    return enc;
  }
  */
  public DifferentialDrive getDrive(){
    return drive;
  }

  /*
  public void resetEncoders(){
    rightEnc.reset();
    leftEnc.reset();
  }
  */

  public void calibrateIMU(){
    imu.calibrate();
  }

  public void resetIMU(){
    imu.reset();
  }

  public double getAngle(){
    return imu.getAngle();
  }

  public double getXHeading() {
    return imu.getAngleX();
  }
  
  public double getYHeading() {
    return imu.getAngleY();
  }
  
  public double getZHeading() {
    return imu.getAngleZ();
  }
  
  public double getXAccel() {
    return imu.getAccelX();
  }
  
  public double getYAccel() {
    return imu.getAccelY();
  }
  
  public double getZAccel() {
    return imu.getAccelZ();
  }
  
  public double getTemp() {
    return imu.getTemperature();
  }
  
  public double getXMag() {
    return imu.getMagX();
  }
  
  public double getYMag() {
    return imu.getMagY();
  }
  
  public double getZMag() {
    return imu.getMagZ();
  }
  
  public double getPressure() {
    return imu.getBarometricPressure();
  }
  
  public double getYaw() {
    return imu.getYaw();
  }
  
  public double getPitch() {
    return imu.getPitch();
  }
  
  public double getRoll() {
    return imu.getRoll();
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
