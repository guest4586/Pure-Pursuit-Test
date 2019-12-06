package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class PIDVelocityControlL extends Command {
  DriveTrain driver;
  double demand = 0;
  public PIDVelocityControlL(double in) {
    driver = DriveTrain.getInstance();
    requires(driver);
    demand = in/0.0005829;

    driver.leftLeader.config_kP(0, 1);
    driver.leftLeader.config_kI(0, 0.001);
    driver.leftLeader.config_kD(0, 2);
    driver.leftLeader.config_kF(0, 0.3);

  }

  @Override
  protected void initialize() {
    driver.leftLeader.set(ControlMode.Velocity, demand);
    System.out.println("L");

  }

  @Override
  protected void execute() {
    driver.leftLeader.set(ControlMode.Velocity, (SmartDashboard.getNumber("setPoint", 0)/0.0005829));

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    driver.leftLeader.set(ControlMode.PercentOutput, 0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}