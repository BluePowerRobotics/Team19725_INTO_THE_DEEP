package org.firstinspires.ftc.teamcode.Vision;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.model.CubeInfo;
import org.opencv.core.Point;

@Config
@TeleOp

public class TEST extends LinearOpMode {
    public static  double X = 100;
    public static  double Y = -10000;
    Point Center = new Point(X,Y);
    CubeInfo a = new CubeInfo(1, Center, 1,1,1,1,1);




    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("info", a.centerpoint.x);
            telemetry.addData("info", a.centerpoint.y);
            telemetry.addData("info", CubeProcessor.ProcessCube(a));
            telemetry.update();
        }
    }
}
