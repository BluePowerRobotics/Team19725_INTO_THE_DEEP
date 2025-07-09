package org.firstinspires.ftc.teamcode.Controllers;

public class ServoRadianCalculator {
    private static ServoRadianCalculator instance;
    public static synchronized ServoRadianCalculator getInstance() {
        if(instance == null){
            instance = new ServoRadianCalculator();
        }
        return instance;
    }

    double a = 153;
    double b = 145;
    double c = 10;
    private double[] result = {0,0,0,0};
    public double[] calculate(double x,double y,double z,double alpha4){
        double theta = 0;
        if (x>0 && y>0){theta = Math.atan(y/x);}
        else if (x<0 && y>0){theta = Math.PI-Math.atan(y/x);}
        else if (x<0 && y<0){theta = Math.PI+Math.atan(y/x);}
        else if (x>0 && y<0){theta = Math.PI*2-Math.atan(y/x);}
        else if (x==0 && y>0){theta = Math.PI*0.5;}
        else if (x==0 && y<0){theta = Math.PI*1.5;}
        double length1 = x-c*Math.sin(alpha4)*Math.cos(theta);
        double length2 = y-c*Math.sin(alpha4)*Math.sin(theta);
        double length3 = z+c*Math.cos(alpha4);
        double length = Math.sqrt(length1*length1+length2*length2+length3*length3);
        double alpha1 = Math.acos((a*a+length*length-b*b)/(2*a*length))+Math.asin(length3/length);
        double alpha2 = Math.acos((a*a+b*b-length*length)/(2*a*b));
        double alpha3 = Math.acos((b*b+length*length-a*a)/(2*b*length))+Math.acos(length3/length)+alpha4;
        result = new double[]{theta,alpha1,alpha2,alpha3};
        return result;
    }
    public double getTheta() {
        return result[0];
    }
    public double getAlpha1() {
        return result[1];
    }
    public double getAlpha2() {
        return result[2];
    }
    public double getAlpha3() {
        return result[3];
    }
}
