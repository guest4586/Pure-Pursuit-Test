package frc.robot.Utils.pure_pursuite;

import java.util.ArrayList;
public class Path extends ArrayList<Waypoint> {

    private double spacing, tolerance;
    public Path(double spacing,double tolerance){
        this.spacing = spacing;
        this.tolerance = tolerance;
    }

    public boolean InjectPoints(){
        if(this.isEmpty())
            return false;
        
        Path newPath = new Path(this.spacing,this.tolerance);
        double distance = 0;
        int j;


        for(int i =  0;i< this.size() -1; i++){
            distance = this.get(i).distance(this.get(i+1));

            j = 0;
            while(j < distance){
                newPath.add(new Waypoint(this.get(i).x + j/distance*(this.get(i+1).x - this.get(i).x) 
                    ,this.get(i).y + j/distance*(this.get(i+1).y - this.get(i).y)
                    ));
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

        Path SmoothedPath = new Path(this.spacing,this.tolerance);
        SmoothedPath.addAll(this);


        double change = tolerance,aux;

        while(change >= tolerance){
            change = 0;
            for(int i = 1;i< this.size()-1;i++){
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
        this.addAll(SmoothedPath);
        return true;
    }


    
    public void initPath(Waypoint[] waypoints){
      for(Waypoint p : waypoints){
        this.add(p);
      }
    }
}