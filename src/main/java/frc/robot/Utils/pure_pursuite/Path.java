package frc.robot.Utils.pure_pursuite;

import java.util.ArrayList;

import frc.robot.Utils.Constants;

public class Path extends ArrayList<Waypoint> {

    private double spacing, tolerance, maxVel;
    public Path(double spacing,double tolerance,double maxVel){
        this.spacing = Constants.SPACING;
        this.tolerance = Constants.TOLERANCE;
        this.maxVel = Constants.MAX_VELOCITY;
    }

    public Waypoint[] toArray(){
        Waypoint[] path = new Waypoint[this.size()];
        for(int i = 0; i< this.size();i++){
            path[i] = this.get(i);
        }
        return path;
    }

    public double getCurv(Waypoint p2,Waypoint p1,Waypoint p3){
        p1.x += 0.0001;
        double k1 = 0.5*(p1.getLengthSq()-p2.getLengthSq())/(p1.x-p2.x);
        double k2 = (p1.y-p2.y)/(p1.x-p2.x);

        double b = 0.5*(p2.x*p2.x - 2*p2.x*k1 + p2.y*p2.y - p3.x*p3.x + 2*p3.x*k1-p3.y*p3.y)/(p3.x*k2 - p3.y+p2.y-p2.x*k2);
        double a = k1 - k2*b;
        double rad = Math.sqrt(Math.pow(p1.x-a,2)+Math.pow(p1.y-b,2));
        if(rad > 10000)
            return 0;
        return 1/rad;
    }

    public void addVelocities(){
        this.get(0).velocity = this.maxVel;
        double curve;
        for(int i = 1; i< this.size()-1;i++){
            curve = getCurv(this.get(i-1),this.get(i),this.get(i+1));
            this.get(i).velocity = Math.min(Constants.VELOCITY_CONST/curve,maxVel);
        }
        this.get(this.size()-1).velocity = 0;
    }

    public boolean InjectPoints(){
        if(this.isEmpty())
            return false;
        
        Path newPath = new Path(this.spacing,this.tolerance,this.maxVel);
        double distance = 0;
        int j;


        for(int i =  0;i< this.size() -1; i++){
            distance = this.get(i).distance(this.get(i+1));

            j = 0;
            while(j < distance){
                newPath.add(new Waypoint(this.get(i).x + j/distance*(this.get(i+1).x - this.get(i).x) 
                    ,this.get(i).y + j/distance*(this.get(i+1).y - this.get(i).y)
                    , 0));
                j += this.spacing;
            }
        }
        this.clear();
        this.addAll(newPath);
        return true;
    }
    public boolean SmoothPoints(double weightData,double weightSmooth){
        if(this.isEmpty())
        return false;

        Path SmoothedPath = new Path(this.spacing,this.tolerance,this.maxVel);
        SmoothedPath.addAll(this);


        double change = this.tolerance,aux;

        while(change >= this.tolerance){
            change = 0;
            for(int i = 1;i< this.size();i++){
                aux =  this.get(i).x;
                SmoothedPath.get(i).x += weightData*(this.get(i).x - SmoothedPath.get(i).x) +
                weightSmooth*(SmoothedPath.get(i-1).x + SmoothedPath.get(i+1).x - 2*this.get(i).x);
                change += Math.abs(aux - SmoothedPath.get(i).x);


                aux =  this.get(i).y;
                SmoothedPath.get(i).y += weightData*(this.get(i).y - SmoothedPath.get(i).y) +
                weightSmooth*(SmoothedPath.get(i-1).y + SmoothedPath.get(i+1).y - 2*this.get(i).y);
 
                change += Math.abs(aux - SmoothedPath.get(i).y);
            }
        }

        this.clear();
        this.addAll(this);
        return true;
    }

    
    public void initPath(Waypoint[] waypoints){
        this.clear();
        for(Waypoint p : waypoints)
          this.add(p);
    }
}