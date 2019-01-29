/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveCargo;

/**
 * Add your docs here.
 */
public class CargoShooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX shootMotor = new WPI_TalonSRX(RobotMap.cargoShooterTalonPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MoveCargo(RobotMap.Cargo.OFF));
  }

  public void pushBall(){
    shootMotor.set(-1);
  }

  public void pullBall(){
    shootMotor.set(1);
  }

  public void stopBall(){
    shootMotor.set(0);
  }

  public double getBall(){
    return shootMotor.get();
  }

  //Provides one singular cargoshooter across all files
  private static CargoShooter instance;
  public static CargoShooter getInstance(){
    if(instance == null)
      instance = new CargoShooter();
    return instance;
  }
}
