package frc.robot.Utils.PID_Controller;


public class PIDController {

    private double kP,kI,kD,kF;
    private double ThreshHold;
    private double DeltaT;

    private double prevError; 
    private double setPoint;
    private double CurrentPos;
    private double OutPut;

    private double I;


    public PIDController(double p,double i,double d,double f){
        this.kP = p;
        this.kD = d;
        this.kI = i;
        this.kF = f;
        this.ThreshHold = 1;
        this.CurrentPos = 0;
        this.I =0;
    }

    public void setDeltaT(double Delta){
        this.DeltaT = Delta;
    }
    public void setThreshHold(double ThreshHold){
        this.ThreshHold = ThreshHold;
    }
    public void setPoint(double setPoint){ 
        this.setPoint = setPoint;
    }

    public void CalculatePID(){
        double error = this.setPoint - this.CurrentPos;
        double P,D;
        P = error*this.kP;
        this.I += (this.DeltaT*(error + prevError)/2)*this.kI;
        D = (error - this.prevError)/this.DeltaT;
        D *= this.kD;
        InRange(P+this.I+D);
    }

    private void InRange(double OutPut){
        //TODO: create a better system for this shit.
        if(OutPut > 1)
            OutPut = 1;
        if(OutPut < -1)
            OutPut = -1;
        this.OutPut = OutPut;
    }

    public double getOutPut(){
        return this.OutPut;
    }




    
    
}