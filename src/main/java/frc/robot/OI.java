package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PIDVelocityControlR;
import frc.robot.commands.run;

public class OI {
  public static Joystick driverJoystick;
  public static JoystickButton x;
  public static JoystickButton a;
  public OI(){
    driverJoystick = new Joystick(0);
    x = new JoystickButton(driverJoystick, 3);
    a = new JoystickButton(driverJoystick, 1);
    x.whileHeld(new run(Robot.path));
    a.whileHeld(new PIDVelocityControlR(0));
  }
}
