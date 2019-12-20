package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Utils.Constants;
import frc.robot.Utils.pure_pursuite.Path;
import frc.robot.Utils.pure_pursuite.Waypoint;
import frc.robot.subsystems.DriveTrain;

public class Purepursuit extends Command {
  private DriveTrain driver;
  private Path path;
  public Purepursuit(Waypoint[] path) {
    driver = DriveTrain.getInstance();
    requires(this.driver);


    this.path = new Path(Constants.SPACING,Constants.TOLERANCE,Constants.MAX_VELOCITY);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
