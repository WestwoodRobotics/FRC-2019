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
import frc.robot.RobotMap.Arm;

import frc.robot.commands.AdjustArm;
import frc.robot.commands.AdjustLift;
import frc.robot.commands.GrabHatch;
import frc.robot.commands.MoveCargo;
import frc.robot.commands.ShiftSlow;
import frc.robot.commands.ToggleArmMode;
import frc.robot.commands.TurnToHatch;


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
  public static final int logitechPort = 0;
  public static final int rJoyPort = 1;
  public static final int lJoyPort = 2;

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

  Button armLower = new JoystickButton(lJoy, 13);
  Button armRaise = new JoystickButton(lJoy, 12);

  Button armPowerRaise = new JoystickButton(lJoy, 11);

  Button turnHatch = new JoystickButton(rJoy, 13);

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
  Joystick throttle = new Joystick(logitechPort);

  Button frontPiston = new JoystickButton(throttle, 2),
         backPiston = new JoystickButton(throttle, 3),
         hatch = new JoystickButton(throttle, 5),
         powerLift = new JoystickButton(throttle, 1);

  
  public OI() {
    //Slow Mode (First Driver)
    rJoyTrigger.whenPressed(new ShiftSlow(false));
    rJoyTrigger.whenReleased(new ShiftSlow(true));

    lJoyTrigger.whenPressed(new ShiftSlow(false));
    lJoyTrigger.whenReleased(new ShiftSlow(true));

    //First Driver Overrides
    rightLower.whenPressed(new GrabHatch(true));

    rightR.whenPressed(new AdjustLift(LiftMode.TOGGLE_FRONT));
    rightL.whenPressed(new AdjustLift(LiftMode.TOGGLE_BACK));

    leftR.whenPressed(new MoveCargo(Cargo.OUT));
    leftR.whenReleased(new MoveCargo(Cargo.OFF));
    
    leftL.whenPressed(new MoveCargo(Cargo.IN));
    leftL.whenReleased(new MoveCargo(Cargo.OFF));

    armLower.whenPressed(new AdjustArm(Arm.DOWN));
    armLower.whenReleased(new AdjustArm(Arm.OFF));

    armRaise.whenPressed(new AdjustArm(Arm.UP));
    armRaise.whenReleased(new AdjustArm(Arm.OFF));

    armPowerRaise.whenPressed(new ToggleArmMode());

    //turnHatch.whenPressed(new TurnToHatch());

    //Second Driver Controls
    logitechY.whenPressed(new AdjustArm(Arm.UP));
    logitechY.whenReleased(new AdjustArm(Arm.OFF));

    logitechA.whenPressed(new AdjustArm(Arm.DOWN));
    logitechA.whenReleased(new AdjustArm(Arm.OFF));
    
    logitechX.whenPressed(new GrabHatch(true));
    
    logitechB.whenPressed(new ToggleArmMode());

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
