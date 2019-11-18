/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ResetOdometry;

public class OI {
  public static Joystick driverJoystick;
  public static JoystickButton x;
  public OI(){
    driverJoystick = new Joystick(0);
    x = new JoystickButton(driverJoystick, 3);
    x.whenPressed(new ResetOdometry());
  }
}
