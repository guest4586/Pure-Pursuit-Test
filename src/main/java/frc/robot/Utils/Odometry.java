package frc.robot.Utils;

public class Odometry extends Vector {


    public Odometry(double x, double y){
       super(x, y); 
    }
    
    public void UpdateRobotPosition(double changeL, double changeR, double robotAngle)
    {
        double dis = (changeL + changeR)/ 2;
        this.x += dis * Math.cos(robotAngle);
        this.y += dis * Math.sin(robotAngle);
    }
    public Vector getRobotPosition(){
        return new Vector(this);
    }

}
