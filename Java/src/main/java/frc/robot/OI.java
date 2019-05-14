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
import frc.robot.RobotMap.E_ARM_POS;
import frc.robot.RobotMap.E_CARGO_MOTION;
import frc.robot.RobotMap.E_LIFT_MODE;
import frc.robot.commands.AdjustLift;
import frc.robot.commands.MoveCargo;
import frc.robot.commands.ReverseDrive;
import frc.robot.commands.SetArm;
import frc.robot.commands.ShiftSlow;


/*
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  // CREATING BUTTONS
  /* A joystick button which is any button on a joystick.
   * You create one by telling it which joystick it's on and which button
   * Ex. Button button = new JoystickButton(joyStick, buttonNumber);
   */
  
  //Controllers accessing the ports
  Joystick j_right = new Joystick(RobotMap.P_OI_JOY_RIGHT);
  Joystick j_left = new Joystick(RobotMap.P_OI_JOY_LEFT);

  Button b_rJoyTrigger = new JoystickButton(j_right, 1);
  Button b_lJoyTrigger = new JoystickButton(j_left, 1);
  
  Button b_rightLower = new JoystickButton(j_right, 2);
  Button b_rightL = new JoystickButton(j_right, 3);
  Button b_rightR = new JoystickButton(j_right, 4);
  
  Button b_leftLower = new JoystickButton(j_left, 2);
  Button b_leftL = new JoystickButton(j_left, 3);
  Button b_leftR = new JoystickButton(j_left, 4);
  
  Button b_leftBaseOut = new JoystickButton(j_left, 10);
  Button b_leftBaseMiddle = new JoystickButton(j_left, 11);
  Button b_leftBaseIn = new JoystickButton(j_left, 12);
  Button b_leftBaseDot = new JoystickButton(j_left, 13);

  //Second Driver
  Joystick j_logitech = new Joystick(RobotMap.P_OI_LOGITECH);

  Button b_logitechA = new JoystickButton(j_logitech, 1),
         b_logitechB = new JoystickButton(j_logitech, 2),
         b_logitechX = new JoystickButton(j_logitech, 3),
         b_logitechY = new JoystickButton(j_logitech, 4),

         b_logitechLBumper = new JoystickButton(j_logitech, 5),
         b_logitechRBumper = new JoystickButton(j_logitech, 6),

         b_logitechRJoyClick = new JoystickButton(j_logitech, 10),
         b_logitechLJoyClick = new JoystickButton(j_logitech, 9);

  
  //-----------------------------------------------------------------------------------------
  // TRIGGERING COMMANDS WITH BUTTONS
  /* Once you have a button, it's trivial to bind it to a button in one of
   three ways:

    * Start the command when the button is pressed and let it run the command
    * until it is finished as determined by it's isFinished method.
    * button.whenPressed(new ExampleCommand());

    * Run the command while the button is being held down and interrupt it once
    * the button is released.
    * button.whileHeld(new ExampleCommand());

    * Start the command when the button is released and let it run the command
    * until it is finished as determined by it's isFinished method.
    * button.whenReleased(new ExampleCommand());
    */

  public OI() {
    //Slow Mode (First Driver)
    b_rJoyTrigger.whenPressed(new ShiftSlow(false));
    b_rJoyTrigger.whenReleased(new ShiftSlow(true));

    b_lJoyTrigger.whenPressed(new ShiftSlow(false));
    b_lJoyTrigger.whenReleased(new ShiftSlow(true));

    //First Driver Overrides
    b_rightR.whenPressed(new AdjustLift(E_LIFT_MODE.TOGGLE_FRONT));
    b_rightL.whenPressed(new AdjustLift(E_LIFT_MODE.TOGGLE_BACK));
    
    b_leftR.whenPressed(new MoveCargo(E_CARGO_MOTION.OUT));
    b_leftR.whenReleased(new MoveCargo(E_CARGO_MOTION.OFF));
    
    b_leftL.whenPressed(new MoveCargo(E_CARGO_MOTION.IN));
    b_leftL.whenReleased(new MoveCargo(E_CARGO_MOTION.OFF));

    b_leftLower.whenPressed(new ReverseDrive());

    //ARM PID
    b_leftBaseDot.whenReleased(new SetArm(E_ARM_POS.BOTTOM));
    b_leftBaseIn.whenReleased(new SetArm(E_ARM_POS.SHIP));
    b_leftBaseMiddle.whenReleased(new SetArm(E_ARM_POS.ROCKET));
    b_leftBaseOut.whenReleased(new SetArm(E_ARM_POS.TOP));
    
    //Second Driver Controls
    b_logitechRBumper.whenPressed(new AdjustLift(E_LIFT_MODE.TOGGLE_BACK));

    b_logitechLBumper.whenPressed(new AdjustLift(E_LIFT_MODE.TOGGLE_FRONT));
  }

  public double getLJoyY(){
    return -j_left.getY();
  } 

  public double getRJoyY(){
    return -j_right.getY();
  } 

  public double getLogitechLJoyY(){
    return j_logitech.getRawAxis(1);
  }

  public double getLogitechRJoyY(){
    return j_logitech.getRawAxis(5);
  }

  /*public int getLPOV(){ //Controls Hatch
    return j_left.getPOV();
  }

  public int getRPOV(){ //Controls Arm
    return j_right.getPOV();
  }*/

  public int getLogitechPOV(){
    return j_logitech.getPOV();
  }

  public boolean getYButton(){
    return b_logitechY.get();
  }

  public boolean getAButton(){
    return b_logitechA.get();
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
