/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.UpdateArm;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.P_ARM_TALON_1),
                       armMotor2 = new WPI_TalonSRX(RobotMap.P_ARM_TALON_2);

  private static double angle = getInstance().getPosition()*(-2*Math.PI/RobotMap.C_TICKS_PER_REV);

  private static RobotMap.E_ARM_POS pos = RobotMap.E_ARM_POS.TOP;

  private static double P = 0,
                        I = 0,
                        D = 0,
                        F = RobotMap.C_HORIZONTAL_VOLTAGE * Math.cos(angle);

  private static double outputVal = 0;

  public Arm(){
    super(P, I, D, F);

    armMotor1.set(ControlMode.Follower, RobotMap.P_ARM_TALON_2);
    armMotor2.set(ControlMode.PercentOutput, 0);

    this.resetEncoder(RobotMap.C_ARM_TOP_POS);
    this.getPIDController().setContinuous(false);

    setPercentTolerance(RobotMap.C_ARM_PERCENT_TOLERANCE);
    setOutputRange(-0.35, 0.15);

    P = SmartDashboard.getNumber("P", 0);
    I = SmartDashboard.getNumber("I", 0);
    D = SmartDashboard.getNumber("D", 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new UpdateArm());
  }

  public void setP(double P){
    this.P = P;
  }
  
  public void setI(double I){
    this.I = I;
  }

  public void setD(double D){
    this.D = D;
  }

  public double getP(){
    return P;
  }

  public double getI(){
    return I;
  }

  public double getD(){
    return D;
  }

  public double getAngle(){
    angle = getInstance().getPosition()*(-2.0*Math.PI/RobotMap.C_TICKS_PER_REV);
    return angle;
  }

  public void updatePID(){
    P = SmartDashboard.getNumber("P", 0);
    I = SmartDashboard.getNumber("I", 0);
    D = SmartDashboard.getNumber("D", 0);
    F = RobotMap.C_HORIZONTAL_VOLTAGE * Math.cos(getAngle());

    this.getPIDController().setP(this.P);
    this.getPIDController().setI(this.I);
    this.getPIDController().setD(this.D);
    this.getPIDController().setF(this.F);

    //System.out.println(toString());
  }

  public void setPos(RobotMap.E_ARM_POS pos){
    int actualPos;
    switch(pos){
      case TOP:
        actualPos = RobotMap.C_ARM_TOP_POS;
        break;
      case ROCKET:
        actualPos = RobotMap.C_ARM_ROCKET_POS;
        break;
      case SHIP:
        actualPos = RobotMap.C_ARM_SHIP_POS;
        break;
      case BOTTOM:
        actualPos = RobotMap.C_ARM_BOTTOM_POS;
        break;
      default:
        actualPos = RobotMap.C_ARM_ROCKET_POS;
    }

    this.setSetpoint(actualPos);
    SmartDashboard.putNumber("Setpoint", this.getSetpoint());
  }
    
  public void setArmSpeed(double speed){
    armMotor2.pidWrite(speed);
  }

  public void brakeArm(){
    armMotor2.set(ControlMode.PercentOutput, 0);
  }

  protected double returnPIDInput() {
    return getEncoder();
  }

  protected void usePIDOutput(double output) {
    output = -output;
    setArmSpeed(output);
    this.outputVal = output;
    //System.out.println(output);
  }

  public double getCurrOutput(){
    return outputVal;
  }

  public double getEncoder(){
    return armMotor2.getSelectedSensorPosition();
  }

  public RobotMap.E_ARM_POS getEnumPos(){
    return this.pos;
  }

  public void resetEncoder(int resetPos){
    armMotor2.setSelectedSensorPosition(resetPos);
  }

  public String toString(){
    return "P: " + this.getPIDController().getP() + 
    "\nI: " + this.getPIDController().getI() + 
    "\nD: " + this.getPIDController().getD() +
    "\nF: " + this.getPIDController().getF() +
    //"\nArm Output: " + this.outputVal + 
    "\nPosition: " + this.getPosition();
  }

  private static Arm instance;
  public static Arm getInstance() {
    if(instance == null) {
      instance = new Arm();
    }
    
    return instance;
  }
}
