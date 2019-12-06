package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.pure_pursuite.*;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  public OI m_oi;
  public DriveTrain m_Driver;
  public Odometry odometry;
  public static PurePursuit controller;
  public static NetworkTable table;
  public static NetworkTableEntry x, y, angle;
  public static Waypoint[] waypoints = {
    new Waypoint(0.0,0.0,3.5),
    new Waypoint(1, 0,69.5),
    new Waypoint(2, 0,540)

  };
  @Override
  public void robotInit() {
    m_Driver = DriveTrain.getInstance();
    m_oi = new OI();
    odometry = new Odometry(0,0);
    table = NetworkTableInstance.getDefault().getTable("odometry");
    x = table.getEntry("x"); 
    y = table.getEntry("y"); 
    angle = table.getEntry("angle");
    controller = new PurePursuit(waypoints); 
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  public void initPath(Waypoint[] waypoints){
  
  
  
  }


  

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

      }


}

