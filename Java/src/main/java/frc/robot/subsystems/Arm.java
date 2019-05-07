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

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.AdjustArm;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.armTalon1Port),
                       armMotor2 = new WPI_TalonSRX(RobotMap.armTalon2Port);

  private static double angle = getInstance().getPosition()*(-2*Math.PI/RobotMap.ticksPerRevolution);

  private static double P = 0,
                        I = 0,
                        D = 0,
                        F = RobotMap.horizontalVoltage * Math.cos(angle);

  private static double outputVal = 0;

  public Arm(){
    super(P, I, D, F);

    armMotor1.set(ControlMode.Follower, RobotMap.armTalon2Port);
    armMotor2.set(ControlMode.PercentOutput, 0);

    this.resetEncoder();
    this.getPIDController().setContinuous(false);

    setPercentTolerance(.1);
    setOutputRange(-0.35, 0.15);

    P = SmartDashboard.getNumber("P", 0);
    I = SmartDashboard.getNumber("I", 0);
    D = SmartDashboard.getNumber("D", 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new AdjustArm(-9000));
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
    angle = getInstance().getPosition()*(-2.0*Math.PI/RobotMap.ticksPerRevolution);
    return angle;
  }

  public void updatePID(){
    P = SmartDashboard.getNumber("P", 0);
    I = SmartDashboard.getNumber("I", 0);
    D = SmartDashboard.getNumber("D", 0);
    F = RobotMap.horizontalVoltage * Math.cos(getAngle());

    this.getPIDController().setP(this.P);
    this.getPIDController().setI(this.I);
    this.getPIDController().setD(this.D);
    this.getPIDController().setF(this.F);

    System.out.println(toString());
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
    System.out.println(output);
  }

  public double getEncoder(){
    return armMotor2.getSelectedSensorPosition();
  }

  public void resetEncoder(){
    armMotor2.setSelectedSensorPosition(-52765);
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
