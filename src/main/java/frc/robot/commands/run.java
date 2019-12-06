
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.pure_pursuite.PurePursuit;
import frc.robot.Utils.pure_pursuite.Vector;
import frc.robot.Utils.pure_pursuite.Waypoint;
import frc.robot.subsystems.DriveTrain;

public class run extends Command {
  Waypoint[] path;
  DriveTrain driver;
  PurePursuit pp;
  public run(Waypoint[] path) {
    this.path = path;
    driver = DriveTrain.getInstance();
    pp = new PurePursuit(this.path);
  }

  @Override
  protected void initialize() {
    driver.resetOdometry();
    driver.setLPO(0);
    driver.setRPO(0);

  }

  @Override
  protected void execute() {
    driver.UpdateOdometry();
    Vector position = driver.getRobotOdometry();
    Waypoint closest = pp.getClosestPoint(this.path, position);
    Vector look = pp.getLookaheadPoint2(position, 0.1, this.path);

    double sc = pp.getSignesCurvature(look, Math.toRadians(driver.getAngle()), position);
    double curv = pp.getCurv(look, .1);
    double ls = pp.getLeftTargetVelocity(closest, curv, 0.55);
    double rs = pp.getRightTargetVelocity(closest, curv, 0.55);
    SmartDashboard.putNumber("gyro", driver.getAngle());
    SmartDashboard.putNumber("curv", sc);
    SmartDashboard.putNumber("LS", ls);
    SmartDashboard.putNumber("RS", rs);
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
