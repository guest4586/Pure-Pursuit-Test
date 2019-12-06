package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class PIDVelocityControlR extends Command {
  DriveTrain driver;
  double demand = 0;

  public PIDVelocityControlR(double in) {
    driver = DriveTrain.getInstance();
    requires(driver);
    demand = in/0.0005829;

    driver.rightLeader.config_kP(0,.8);
    driver.rightLeader.config_kI(0, 0.0025);
    driver.rightLeader.config_kD(0, 24);
    driver.rightLeader.config_kF(0, 0.3);
    SmartDashboard.putNumber("setPoint", 0);
  } 

  @Override
  protected void initialize() {
    driver.rightLeader.set(ControlMode.Velocity, demand);
  }

  @Override
  protected void execute() {
    driver.rightLeader.set(ControlMode.Velocity, -SmartDashboard.getNumber("setPoint", .3)/0.0005829);
    driver.leftLeader.set(ControlMode.Velocity, SmartDashboard.getNumber("setPoint", .3)/0.0005829);
    


  }
  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    driver.rightLeader.set(ControlMode.PercentOutput, 0);
    driver.leftLeader.set(ControlMode.PercentOutput, 0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
