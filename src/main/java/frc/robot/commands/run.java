
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.pure_pursuite.Path;
import frc.robot.Utils.pure_pursuite.PurePursuit;
import frc.robot.Utils.pure_pursuite.Vector;
import frc.robot.Utils.pure_pursuite.Waypoint;
import frc.robot.subsystems.DriveTrain;

public class run extends Command {
  private Path path;
  private DriveTrain driver;
  private PurePursuit pp;
  private double lookDistance = 0.3;
  public run(Path path) {
    this.path = path;
    this.driver = DriveTrain.getInstance();
  }

  @Override
  protected void initialize() {
    // this.path.addVelocities(5);
    driver.resetOdometry();
    driver.setLPO(0);
    this.pp = new PurePursuit(this.path,this.lookDistance);
    driver.setRPO(0);
  }

  @Override
  protected void execute() {
    driver.UpdateOdometry();
    double angle = driver.getAngle();
    Vector position = driver.getRobotOdometry();
    Waypoint closest = pp.getClosestPoint(position);
    Vector look = pp.getLookaheadPoint(position);

    double sc = pp.getCurvature(angle,position, look);   
    // double sc = Math.abs(curv)*pp.getSignesCurvature(look, Math.toRadians(driver.getAngle()), position);
    double ls = pp.getLeftTargetVelocity(closest, sc, 0.55);
    double rs = pp.getRightTargetVelocity(closest, sc, 0.55);
    SmartDashboard.putNumber("gyro", driver.getAngle());
    SmartDashboard.putNumber("curveture", sc);
    SmartDashboard.putNumber("LS",ls);
    SmartDashboard.putNumber("RS",rs);
  }

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
