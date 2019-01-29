/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.sun.java.swing.plaf.windows.TMSchema.Control;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.armTalon1Port),
                       armMotor2 = new WPI_TalonSRX(RobotMap.armTalon2Port);

  private SpeedControllerGroup group = new SpeedControllerGroup(armMotor1, armMotor2);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    armMotor2.set(ControlMode.Follower, RobotMap.armTalon1Port);
  }

  public void setArm(double value){
    armMotor1.set(value);
  }

  private static Arm instance;
  public static Arm getInstance() {
    if(instance == null) {
      instance = new Arm();
    }
    
    return instance;
  }
}
