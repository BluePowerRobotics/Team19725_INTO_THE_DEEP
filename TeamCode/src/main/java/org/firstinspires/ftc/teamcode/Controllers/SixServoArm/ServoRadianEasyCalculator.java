package org.firstinspires.ftc.teamcode.Controllers.SixServoArm;

public class ServoRadianEasyCalculator {
    private static ServoRadianEasyCalculator instance;
    public static synchronized ServoRadianEasyCalculator getInstance() {
        if(instance == null){
            instance = new ServoRadianEasyCalculator();
        }
        return instance;
    }

    double a = 153;
    double b = 143;
    double c = 125;//useless
    private double[] result = {0,0,0,0};
    //去除z轴方向控制，保持为0

    //水平向上为0，顺时针增加，[0Pi,Pi]
    public double[] calculate(double x,double y,double clipHeadingRadian){
        double radian0 = 0;
        if (x>0 && y>0){radian0 = Math.atan(y/x);}
        else if (x<0 && y>0){radian0 = Math.PI-Math.atan(y/x);}
        else if (x<0 && y<0){radian0 = Math.PI+Math.atan(y/x);}
        else if (x>0 && y<0){radian0 = Math.PI*2-Math.atan(y/x);}
        else if (x==0 && y>0){radian0 = Math.PI*0.5;}
        else if (x==0 && y<0){radian0 = Math.PI*1.5;}
        double r = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        double radian1 = Math.acos((Math.pow(a,2)+Math.pow(r,2)-Math.pow(b,2))/(2*a*r));
        //double radian1 = Math.PI/2;
        double radian2 = Math.acos((Math.pow(a,2)+Math.pow(b,2)-Math.pow(r,2))/(2*a*b));
        double radian3 = Math.acos((Math.pow(b,2)+Math.pow(r,2)-Math.pow(a,2))/(2*b*r))+1.5*Math.PI-clipHeadingRadian;
        result = new double[]{radian0,radian1,radian2,radian3};
        return result;
    }
    public double getRadian0() {
        return result[0];
    }
    public double getRadian1() {
        return result[1];
    }
    public double getRadian2() {
        return result[2];
    }


    public double getRadian3() {
        return result[3];
    }
}
