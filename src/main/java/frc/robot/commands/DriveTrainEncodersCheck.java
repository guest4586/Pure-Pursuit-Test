/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainEncodersCheck extends Command {
  DriveTrain driver;
  int counter;
  
  public DriveTrainEncodersCheck() {
    driver = DriveTrain.getInstance();
    requires(driver);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    counter = 0;
    setTimeout(5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!isTimedOut()){
      driver.setLeftSpeed(0.7);
      driver.setRightSpeed(0.7);
      SmartDashboard.putNumber("Encoder Delta", Math.abs(driver.getLeftEncoderPosition() - driver.getRightEncoderPosition()));
    }
    else{
      driver.setLeftSpeed(0);
      driver.setRightSpeed(0);
      driver.resetOdometry();
      counter++;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return counter == 5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driver.setLeftSpeed(0);
    driver.setRightSpeed(0);
  }

  @Override
  protected void interrupted() {
  }
}
