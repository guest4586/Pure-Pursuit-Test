package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.run;

public class OI {
  public static Joystick driverJoystick;
  public static JoystickButton x;
  public OI(){
    driverJoystick = new Joystick(0);
    x = new JoystickButton(driverJoystick, 3);
    x.whileHeld(new run(Paths.pathTest));
  }
}
