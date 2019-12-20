package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.DriveTrain;

public class DriveArcade extends Command {
  private DriveTrain m_DriveTrain;
  private Joystick Input;

  private double rot,speed;
  public DriveArcade() {
    this.m_DriveTrain = DriveTrain.getInstance();
    requires(this.m_DriveTrain);
    Input = OI.driverJoystick;
    
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    this.rot = this.Input.getRawAxis(4)*SmartDashboard.getNumber("Max speed",0.7);
    this.speed = this.Input.getRawAxis(1)*SmartDashboard.getNumber("Max speed",1);
    SmartDashboard.putNumber("speed", this.speed);
    m_DriveTrain.ArcadeDrive(rot, speed);
  }

  

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
