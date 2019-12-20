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
  public static Path path;
  public DriveTrain m_Driver;
  public Odometry odometry;
  public static PurePursuit controller;
  public static NetworkTable table;
  public static NetworkTableEntry x, y, angle;
  
  
  public static Waypoint[] waypoints = {
    new Waypoint(0,0,1.5),
    new Waypoint(0.8, 0, 1.5),
    new Waypoint(1.6,-0.4,1.5),
    new Waypoint(2.8,0,1.3),
    new Waypoint(3.5, 0,0)
  };

  @Override
  public void robotInit() {
    m_Driver = DriveTrain.getInstance();
    odometry = new Odometry(0,0);
    table = NetworkTableInstance.getDefault().getTable("odometry");
    x = table.getEntry("x"); 
    y = table.getEntry("y");
    path = new Path(0.1,0,5);
    path.initPath(waypoints); 
    angle = table.getEntry("angle");
    controller = new PurePursuit(path,0.1); 
    m_oi = new OI();
    SmartDashboard.putNumber("setSpeed",0);
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

