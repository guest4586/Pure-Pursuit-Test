package frc.robot.Utils;

public class PurePursuit {
    private int lastClosestPoint;
    private double lastLookaheadindex;
    private Vector lastLookaheadPoint;

    public PurePursuit(Waypoint[] path){
        this.lastClosestPoint = 0;
        this.lastLookaheadindex = 0;
        this.lastLookaheadPoint = path[0];
    }


    public Waypoint getClosestPoint(Waypoint[] path, Vector robotPos){
        int closest = lastClosestPoint + 1;
        for(int i = lastClosestPoint; i < path.length; i++){
            if(robotPos.distance(path[i])< robotPos.distance(path[closest])) closest = i;
        }

        this.lastClosestPoint = closest;
        return path[closest];
    }

    public Vector getLookaheadPoint(Vector robotPos, double lookaheadDistance, Waypoint[] path){
        Vector lookaheadPoint = null; double lookaheadindex = 0;
        for(int i = (int)lastLookaheadindex; i < path.length-1; i++){
            Vector E = path[i]; Vector L = path[i+1];
            Vector d = Vector.subtract(L, E);
            Vector f = Vector.subtract(E, robotPos);
            double a = Vector.dot(d, d); double b = 2*(Vector.dot(d, f)); double c = Vector.dot(f, f) - (lookaheadDistance*lookaheadDistance);
            double discriminant = b*b - 4*a*c;

            if(discriminant >= 0){
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant)/(2*a*c);
                double t2 = (-b + discriminant)/(2*a*c);
                
                if(t1 >= 0 && t1 <= 1){
                    lookaheadPoint = Vector.add(E, d.getMultiplied(t1));
                    lookaheadindex = t1 + i;
                    if(lookaheadindex > lastLookaheadindex){
                        lastLookaheadindex = lookaheadindex;                        
                        return lookaheadPoint;
                    }
                }
                if(t2 >= 0 && t2 <= 1){
                    lookaheadPoint = Vector.add(E, d.getMultiplied(t2));
                    lookaheadindex = t2 + i;
                    if(lookaheadindex > lastLookaheadindex){
                        lastLookaheadindex = lookaheadindex;                        
                        return lookaheadPoint;
                    }
                }
            }
        }

        return lastLookaheadPoint;
    }

    public double getSignesCurvature(Vector lookaheadpoint, double robotAngle, Vector robotPos){
        return Math.signum(Math.sin(robotAngle)*(lookaheadpoint.x - robotPos.x) - Math.cos(robotAngle)*(lookaheadpoint.y - robotPos.y));
    }

    public double getRightTargetVelocity(Waypoint closestWaypoint, double curvature, double trackWidth){
        return closestWaypoint.velocity* (2 + curvature * trackWidth)/2;
    }
    
    public double getLeftTargetVelocity(Waypoint closestWaypoint, double curvature, double trackWidth){
        return closestWaypoint.velocity* (2 - curvature * trackWidth)/2;
    }

}
