/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.AdjustArm;
import frc.robot.commands.SetArmPos;
import frc.robot.commands.UpdateArm;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.armTalon1Port),
                       armMotor2 = new WPI_TalonSRX(RobotMap.armTalon2Port);

  private boolean powerMode = false;
  
  //private static double kP = 0;
  //private static double kI = 0;
  //private static double kD = 0;

  int tolerance = 10;

  private static RobotMap.ArmEnum relativePos;
  private static double absolutePos = 0;

  public Arm(){
    armMotor1.setNeutralMode(NeutralMode.Brake);
    armMotor2.setNeutralMode(NeutralMode.Brake);;

    powerMode = false;

    /*armMotor2.configFactoryDefault();
    
    armMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    armMotor2.configReverseSoftLimitThreshold(RobotMap.encoderBackwardLimit);
    armMotor2.configReverseSoftLimitEnable(true);

    armMotor2.config_kP(0, kP, 0); 
    armMotor2.config_kI(0, kI, 0);
    armMotor2.config_kD(0, kD, 0);

    armMotor2.configAllowableClosedloopError(0, tolerance, 0);

    armMotor2.selectProfileSlot(0, 0);*/

    armMotor1.set(ControlMode.Follower, RobotMap.armTalon2Port);
    
    resetEncoder();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new UpdateArm());
  }

  public void setArm(RobotMap.ArmEnum pos){
    relativePos = pos;
    if(pos.equals(RobotMap.ArmEnum.TOP)){
      absolutePos = RobotMap.armTopPos;
    }
    else if(pos.equals(RobotMap.ArmEnum.MIDDLE)){
      absolutePos = RobotMap.armMiddlePos;
    }
    else if(pos.equals(RobotMap.ArmEnum.BOTTOM)){
      absolutePos = RobotMap.armBottomPos;
    }
  }

  public void setArm(int increment){
    absolutePos += increment;
  }

  public RobotMap.ArmEnum getArm(){
    return relativePos;
  }

  public void updateArm(){
    //armMotor2.set(ControlMode.MotionMagic, absolutePos);
  }
  
  public int getEncoder(){
    return armMotor2.getSensorCollection().getQuadraturePosition();
  }

  public void resetEncoder(){
    armMotor2.getSensorCollection().setQuadraturePosition(0, 100);
  }

  public double getAbsolutePos(){
    return absolutePos;
  }

  // Power Mode Commands
  public void setArmSpeed(double speed){
    armMotor2.set(ControlMode.PercentOutput, speed);
    armMotor1.set(ControlMode.PercentOutput, -speed);
  }

  public void brakeArm(){
    /*if(!powerMode){
      setArmSpeed(.2);
    }
    else
      setArmSpeed(-0.1);*/
  }

  public void togglePowerMode(){
    powerMode = !powerMode;
  }

  public boolean getPowerMode(){
    return powerMode;
  }

  public void setPowerMode(boolean powerMode){
    this.powerMode = powerMode;
  }

  private static Arm instance;
  public static Arm getInstance() {
    if(instance == null) {
      instance = new Arm();
    }
    
    return instance;
  }
}
