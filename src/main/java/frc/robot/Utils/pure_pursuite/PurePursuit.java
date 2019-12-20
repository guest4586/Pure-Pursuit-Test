package frc.robot.Utils.pure_pursuite;

import java.util.ArrayList;


import frc.robot.Utils.Constants;

public class PurePursuit {
    private Odometry position;
    private double lookDistance;
    private int lastClosestPoint;
    public double lastLookaheadindex;
    private Vector lastLookaheadPoint;
    private ArrayList<Waypoint> path;

    public PurePursuit(ArrayList<Waypoint> Path){
        this.lookDistance = Constants.LOOK_DISTANCE;
        this.path = path;
        this.position = Odometry.getInstance();
        resetPurePursuit();
    }
    public void resetPurePursuit(){
        this.position.Reset();
        this.lastClosestPoint = 0;
        this.lastLookaheadindex = 0;
        this.lastLookaheadPoint = path.get(0);
    }

    /**
     * @return the closest point to the robot
     */
    public Waypoint getClosestPoint(){
        int closest = lastClosestPoint + 1;
        int length = path.size();
        if(lastClosestPoint >= length -1) {
             return path.get(length-1);}

        for(int i = lastClosestPoint; i < path.size(); i++){
            if(this.position.distance(path.get(i))< this.position.distance(path.get(closest))) closest = i;
        }

        this.lastClosestPoint = closest;
        return path.get(closest);
    }


    /**
     * @param RobotAngle
     * @param pos robots position
     * @param look look ahead point 
     * @return the path curvature
     */
    public double getCurvature(double RobotAngle,Vector look){
        RobotAngle = Math.toRadians(RobotAngle);
        Vector delta = Vector.subtract(look,this.position);
        double beta = delta.getAngle(),toAngle = beta - RobotAngle;
        Vector relitiVector = new Vector();
        relitiVector = Vector.toCartesian(this.lookDistance, toAngle);
        return (2*relitiVector.y)/Math.pow(this.lookDistance,2);
    }

    public double geteLeftDesiredVelocity(double targetV, double curveture){
        return targetV*(2+(curveture * Constants.TRACK_WIDTH))/2;
    }
    public double getRightDesiredVelocity(double targetV, double curveture){
        return targetV*(2-(curveture * Constants.TRACK_WIDTH))/2;
    }

    /**
     * this function gets the closest look ahead point in the path
     * @link source https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899      
     * @param pos
     * @param path
     * @return the look ahead point
     */
    public Vector getLookaheadPoint() {
        Vector lookahead = null;
        double length;
        int size = path.size();
        // iterate through all pairs of points
        for (int i = 0; i < size - 1; i++) {
            // form a segment from each two adjacent points
            Vector E = path.get(i);
            Vector L = path.get(i + 1);

            Vector d = Vector.subtract(L,E);
            Vector f = Vector.subtract(E,this.position);
            length = d.getLength();
            double a = d.dot(d);
            double b = 2*f.dot(d);
            double c = f.dot(f) - Math.pow(this.lookDistance,2);

            double discriminant = b*b - 4*a*c;

            if(discriminant >= 0){

                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant)/(2*a);
                double t2 = (-b + discriminant)/(2*a);
                if(t1>= 0 && t1<=1){
                    lookahead = new Vector(E);
                    d.normalize();
                    d.multiply(t1*length);
                    lookahead.add(d);
                }
                if(t2>= 0 && t2<=1){
                    d.normalize();
                    lookahead = new Vector(E);
                    d.multiply(t2*length);
                    lookahead.add(d);
                }
            }
        }

        if(lookahead == null) lookahead = (Vector)path.get(0);
        return lookahead;

    }

    /**
     * this function has been fully replaced by the getCurvature() Method
     * @return 1 if the curv is to the right and -1 if its to the left
     */
    public double getSignesCurvature(Vector lookaheadpoint, double robotAngle){
        return Math.signum(Math.sin(robotAngle)*(lookaheadpoint.x - this.position.x) - Math.cos(robotAngle)*(lookaheadpoint.y - this.position.y));
    }
    public void setLastLookAHeadPoint(Vector lookPoint){
        this.lastLookaheadPoint = lookPoint;
    }
}
