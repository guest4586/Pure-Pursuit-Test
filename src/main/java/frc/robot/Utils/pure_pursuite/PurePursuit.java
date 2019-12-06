package frc.robot.Utils.pure_pursuite;

public class PurePursuit {
    private int lastClosestPoint;
    public double lastLookaheadindex;
    private Vector lastLookaheadPoint;
    private Waypoint[] path;

    public PurePursuit(Waypoint[] path){
        this.path = path;
        this.lastClosestPoint = 0;
        this.lastLookaheadindex = 0;
        this.lastLookaheadPoint = path[0];
    }
    public void resetPurePursuit(){
        this.lastClosestPoint = 0;
        this.lastLookaheadindex = 0;
        this.lastLookaheadPoint = path[0];
    }

    public Waypoint getClosestPoint(Waypoint[] path, Vector robotPos){
        int closest = lastClosestPoint + 1;

        if(lastClosestPoint >= path.length -1) {
             return path[path.length-1];}

        for(int i = lastClosestPoint; i < path.length; i++){
            if(robotPos.distance(path[i])< robotPos.distance(path[closest])) closest = i;
        }

        this.lastClosestPoint = closest;
        return path[closest];
    }
    public Vector getLookaheadPoint3(Vector pos, double lookDistance,Waypoint[] path) {

        Vector lookahead = lastLookaheadPoint;
        double length, fractionalIndex = 0;
        // iterate through all pairs of points
        for (int i = (int) lastLookaheadindex; i < path.length - 1; i++) {
            // form a segment from each two adjacent points
            Vector E = path[i];
            Vector L = path[i + 1];

            Vector d = new Vector(L); d.subtract(E);
            Vector f = new Vector(E); f.subtract(pos);
            length = d.getLength();
            double a = d.dot(d);
            double b = 2*f.dot(d);
            double c = f.dot(f) - Math.pow(lookDistance,2);

            double discriminant = b*b - 4*a*c;

            if(discriminant >= 0){

                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant)/(2*a);
                double t2 = (-b + discriminant)/(2*a);
                if(t1>= 0 && t1<=1){
                    fractionalIndex = i + t1;
                    if(fractionalIndex > lastLookaheadindex){
                    lookahead = new Vector(E);}
                    d.normalize();
                    d.multiply(t1*length);
                    lookahead.add(d);
//                    System.out.println(t1);
                }
                if(t2>= 0 && t2<=1){
                    d.normalize();
                    fractionalIndex = i + t2;
                    if(fractionalIndex > lastLookaheadindex){
                        lookahead = new Vector(E);}
                    d.multiply(t2*length);
                    lookahead.add(d);
                    // System.out.println("t2  = "+t2);
                }
            }
        }
        lastLookaheadindex = fractionalIndex;
        lastLookaheadPoint = lookahead;
        return lookahead;
    }
    public static Vector getLookaheadPoint2(Vector pos, double lookDistance,Waypoint[] path) {
        Vector lookahead = null;
//        lookDistance *=0.5;
        double length, fractionalIndex = 0;
        // iterate through all pairs of points
        for (int i = 0; i < path.length - 1; i++) {
            // form a segment from each two adjacent points
            Vector E = path[i];
            Vector L = path[i + 1];

            Vector d = new Vector(L); d.subtract(E);
            Vector f = new Vector(E); f.subtract(pos);
            length = d.getLength();
            double a = d.dot(d);
            double b = 2*f.dot(d);
            double c = f.dot(f) - Math.pow(lookDistance,2);

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
//                    System.out.println(t1);
                }
                if(t2>= 0 && t2<=1){
                    d.normalize();
                    lookahead = new Vector(E);
                    d.multiply(t2*length);
                    lookahead.add(d);
                    // System.out.println("t2  = "+t2);
                }
            }
        }

        if(lookahead == null) lookahead = (Vector)path[0];
        return lookahead;

    }

    public static Vector getLookaheadPoint(Vector pos, double lookDistance,Waypoint[] path) {
        Vector lookahead = null;
        double length;
        // iterate through all pairs of points
        for (int i = 0; i < path.length - 1; i++) {
            // form a segment from each two adjacent points
            Vector E = path[i];
            Vector L = path[i+1];

            Vector d = new Vector(L); d.subtract(E);
            Vector f = new Vector(E); f.subtract(pos);
            length = d.getLength();
            double a = d.dot(d);
            double b = 2*f.dot(d);
            double c = f.dot(f) - lookDistance*lookDistance;

            double discriminant = b*b - 4*a*c;

            if(discriminant >= 0){
            }
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant)/(2*a);
                double t2 = (-b + discriminant)/(2*a);
                lookahead = new Vector(E);
                d.normalize();
                if(t1>= 0 && t1<=1){
                    d.multiply(t1*length);
                    lookahead.add(d);
                    // System.out.print("t1 = "+t1);

                }
                d.normalize();
                lookahead = new Vector(E);
                if(t2>= 0 && t2<=1){
                        d.multiply(t2*length);
                        lookahead.add(d);
                    // System.out.println(" t2  = "+t2);
                }
            }
            if(lookahead == null) return path[0];
            return lookahead;

        }

    public double getSignesCurvature(Vector lookaheadpoint, double robotAngle, Vector robotPos){
        return Math.signum(Math.sin(robotAngle)*(lookaheadpoint.x - robotPos.x) - Math.cos(robotAngle)*(lookaheadpoint.y - robotPos.y));
    }

    public double getCurv(Vector lookahead,double lookDis){return (2*lookahead.x)/(lookDis*lookDis);}

    public double getRightTargetVelocity(Waypoint closestWaypoint, double curvature, double trackWidth){
        return closestWaypoint.velocity* (2 + curvature * trackWidth)/2;
    }
    
    public double getLeftTargetVelocity(Waypoint closestWaypoint, double curvature, double trackWidth){
        return closestWaypoint.velocity* (2 - curvature * trackWidth)/2;
    }

}
