/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.AdjustArm;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.armTalon1Port),
                       armMotor2 = new WPI_TalonSRX(RobotMap.armTalon2Port);

  private SpeedControllerGroup group = new SpeedControllerGroup(armMotor1, armMotor2);

  public Arm(){
    armMotor1.setNeutralMode(NeutralMode.Brake);
    armMotor2.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new AdjustArm(RobotMap.Arm.OFF));
  }

  public void setArm(double value){
    group.set(value);
  }

  public void brakeArm(){
    group.stopMotor();
  }

  private static Arm instance;
  public static Arm getInstance() {
    if(instance == null) {
      instance = new Arm();
    }
    
    return instance;
  }
}
