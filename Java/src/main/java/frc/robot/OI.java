/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.Cargo;
import frc.robot.RobotMap.LiftMode;
import frc.robot.RobotMap.ArmEnum;

import frc.robot.subsystems.Arm;

import frc.robot.commands.AdjustLift;
import frc.robot.commands.GrabHatch;
import frc.robot.commands.MoveCargo;
import frc.robot.commands.ReverseDrive;
import frc.robot.commands.ShiftSlow;
import frc.robot.commands.TurnToHatch;
import frc.robot.subsystems.DriveTrain;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  //Ports
  public static final int logitechPort = 3;
  public static final int rJoyPort = 0;
  public static final int lJoyPort = 1;

  //Controllers accessing the ports
  Joystick rJoy = new Joystick(rJoyPort);
  Joystick lJoy = new Joystick(lJoyPort);

  Button rJoyTrigger = new JoystickButton(rJoy, 1);
  Button lJoyTrigger = new JoystickButton(lJoy, 1);
  
  Button rightR = new JoystickButton(rJoy, 4);
  Button rightL = new JoystickButton(rJoy, 3);
  Button rightLower = new JoystickButton(rJoy, 2);
  
  Button leftR = new JoystickButton(lJoy, 4);
  Button leftL = new JoystickButton(lJoy, 3);
  Button leftLower = new JoystickButton(lJoy, 2);

  //Second Driver
  Joystick logitech = new Joystick(logitechPort);

  Button logitechA = new JoystickButton(logitech, 1),
         logitechB = new JoystickButton(logitech, 2),
         logitechX = new JoystickButton(logitech, 3),
         logitechY = new JoystickButton(logitech, 4),

         logitechLBumper = new JoystickButton(logitech, 5),
         logitechRBumper = new JoystickButton(logitech, 6),

         logitechRJoyClick = new JoystickButton(logitech, 10),
         logitechLJoyClick = new JoystickButton(logitech, 9);

  //Second second driver
  /*Joystick throttle = new Joystick(logitechPort);

  Button frontPiston = new JoystickButton(throttle, 2),
         backPiston = new JoystickButton(throttle, 3),
         hatch = new JoystickButton(throttle, 5),
         powerLift = new JoystickButton(throttle, 1);*/

  
  public OI() {
    //Slow Mode (First Driver)
    rJoyTrigger.whenPressed(new ShiftSlow(false));
    rJoyTrigger.whenReleased(new ShiftSlow(true));

    lJoyTrigger.whenPressed(new ShiftSlow(false));
    lJoyTrigger.whenReleased(new ShiftSlow(true));

    //First Driver Overrides
    rightR.whenPressed(new AdjustLift(LiftMode.TOGGLE_FRONT));
    rightL.whenPressed(new AdjustLift(LiftMode.TOGGLE_BACK));

    leftR.whenPressed(new MoveCargo(Cargo.OUT));
    leftR.whenReleased(new MoveCargo(Cargo.OFF));
    
    leftL.whenPressed(new MoveCargo(Cargo.IN));
    leftL.whenReleased(new MoveCargo(Cargo.OFF));

    leftLower.whenPressed(new ReverseDrive());

    //Second Driver Controls
    logitechRBumper.whenPressed(new AdjustLift(LiftMode.TOGGLE_BACK));

    logitechLBumper.whenPressed(new AdjustLift(LiftMode.TOGGLE_FRONT));
  }

  public double getLJoyY(){
    return -lJoy.getY();
    //return 0;
  } 

  public double getRJoyY(){
    return -rJoy.getY();
    //return 0;
  } 

  public double getLogitechLJoyY(){
    return logitech.getRawAxis(1);
    //return -throttle.getRawAxis(2);
  }

  public double getLogitechRJoyY(){
    return logitech.getRawAxis(5);
  }

  public int getLPOV(){ //Controls Hatch
    return lJoy.getPOV();
  }

  public int getRPOV(){ //Controls Arm
    return rJoy.getPOV();
  }

  public int getLogitechPOV(){
    return logitech.getPOV();
  }

  public boolean getYButton(){
    return logitechY.get();
  }

  public boolean getAButton(){
    return logitechA.get();
  }

  /*public double getZRotate(){
    return -throttle.getRawAxis(5);
  }*/

  //Provides for one singular operator interface across all files
  private static OI instance;
  public static OI getInstance() {
    if(instance == null) {
      instance = new OI();
    }
    
    return instance;
  }
}
