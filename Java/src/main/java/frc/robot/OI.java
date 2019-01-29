/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap.Cargo;
import frc.robot.RobotMap.LiftMode;
import frc.robot.commands.AdjustLift;
import frc.robot.commands.GrabHatch;
import frc.robot.commands.MoveCargo;
import frc.robot.commands.ShiftSlow;


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
  public static final int xBoxPort = 0;
  public static final int rJoyPort = 1;
  public static final int lJoyPort = 2;

  //Controllers accessing the ports
  Joystick rJoy = new Joystick(rJoyPort);
  Joystick lJoy = new Joystick(lJoyPort);
  
  Button rJoyTrigger = new JoystickButton(rJoy, 1);
  Button lJoyTrigger = new JoystickButton(lJoy, 1);
  
  Button rightR = new JoystickButton(rJoy, 4);
  Button rightL = new JoystickButton(rJoy, 3);
  
  Button leftR = new JoystickButton(lJoy, 4);
  Button leftL = new JoystickButton(lJoy, 3);
  Button leftLower = new JoystickButton(lJoy, 2);

  public OI() {
    rJoyTrigger.whenPressed(new ShiftSlow(true));
    rJoyTrigger.whenReleased(new ShiftSlow(false));

    lJoyTrigger.whenPressed(new ShiftSlow(true));
    lJoyTrigger.whenReleased(new ShiftSlow(false));

    rightR.whenPressed(new AdjustLift(LiftMode.TOGGLE_FRONT));
    rightL.whenPressed(new AdjustLift(LiftMode.TOGGLE_BACK));

    leftR.whenPressed(new MoveCargo(Cargo.OUT));
    leftL.whenPressed(new MoveCargo(Cargo.IN));

    leftLower.whenPressed(new GrabHatch(true));
  }

  public double getLJoyY(){
    return lJoy.getY();
  }

  public double getRJoyY(){
    return rJoy.getY();
  }

  //Provides for one singular operator interface across all files
  private static OI instance;
  public static OI getInstance() {
    if(instance == null) {
      instance = new OI();
    }
    
    return instance;
  }
}
