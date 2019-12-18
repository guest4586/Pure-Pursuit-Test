package frc.robot.Utils.pure_pursuite;

public class Odometry extends Vector {


    public Odometry(double x, double y){
       super(x, y); 
    }
    
  public double encoderToMeters(double in){
    return (0.1524*Math.PI*in)/4096;
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

}
