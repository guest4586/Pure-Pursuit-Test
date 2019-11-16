/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utils;

/**
 * Add your docs here.
 */
public class Waypoint extends Vector{
    public double velocity;


    public Waypoint(double x, double y, double v){
        super(x, y);
        this.velocity = v;
    }
    public Waypoint(double x, double y){
        super(x,y);
    }

}
