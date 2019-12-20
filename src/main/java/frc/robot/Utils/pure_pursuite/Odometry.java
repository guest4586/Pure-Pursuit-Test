package frc.robot.Utils.pure_pursuite;

public class Odometry extends Vector {

  private static Odometry instance;
  private Odometry(double x, double y){
    super(x, y); 
  }
    
  private double encoderToMeters(double in){
    return (0.1524*Math.PI*in)/4096;
  }

  public static Odometry getInstance(){
    if(instance == null) instance = new Odometry(0, 0);
    return instance;
  }
  
  public void UpdateRobotPosition(double changeL, double changeR, double robotAngle)
  {
      double angle = Math.toRadians(robotAngle);
      double dis = encoderToMeters((changeL + changeR)/ 2);
      this.x += dis * Math.cos(angle);
      this.y += dis * Math.sin(angle);
  }
  public Vector getRobotPosition(){
    return new Vector(this);
  }
  public void Reset(){
    instance = new Odometry(0,0);
  }

}
