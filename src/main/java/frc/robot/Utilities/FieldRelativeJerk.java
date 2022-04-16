package frc.robot.Utilities;

public class FieldRelativeJerk {
    public double jx;
    public double jy;
    public double jz;

    public FieldRelativeJerk(double jx, double jy, double jz){
        this.jx = jx;
        this.jy = jy;
        this.jz = jz;
    }

    public FieldRelativeJerk(FieldRelativeAccel newAccel, FieldRelativeAccel oldAccel, double time){
        this.jx = (newAccel.ax-oldAccel.ax)/time;
        this.jy = (newAccel.ay-oldAccel.ay)/time;
        this.jz = (newAccel.alpha-oldAccel.alpha)/time;

        if(Math.abs(this.jx) > 3.0){
            this.jx = 3.0*Math.signum(this.jx);
        }
        if(Math.abs(this.jy) > 3.0){
            this.jy = 3.0*Math.signum(this.jy);
        }
        if(Math.abs(this.jz) > 2*Math.PI){
            this.jz = 2*Math.PI*Math.signum(this.jz);
        }
    }

    public FieldRelativeJerk(){
        this.jx = 0.0;
        this.jy = 0.0;
        this.jz = 0.0;
    }
    
}
