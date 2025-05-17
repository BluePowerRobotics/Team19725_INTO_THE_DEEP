package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Auto_2025_Right", group = "Autonomous")
public class Auto2025Right extends LinearOpMode {
    GoBildaPinpointDriver odo;
    boolean ifblue = true;



        //public final GoBildaPinpointDriver odo;



        //@Override
        public void runOpMode() {
            AddTele step1  = new AddTele("step1", 1, telemetry);
            AddTele step2  = new AddTele("step2", 2, telemetry);
            AddTele step3  = new AddTele("step3", 3, telemetry);
            AddTele step4  = new AddTele("step4", 4, telemetry);
            AddTele step5  = new AddTele("step5", 5, telemetry);

            Pose2d initialPoseRight = new Pose2d(32.75, -64.575, Math.toRadians(180.00));



            double ClimbHeading = Math.toRadians(270);


            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseRight);

            Arm armController = new Arm(hardwareMap,telemetry);
            //AutoAlignment autofollow = new AutoAlignment(hardwareMap, drive, telemetry,ifblue);
            //旋转角度的cos和sin的值（0，1）-> 90°



            TrajectoryActionBuilder Climb = drive.actionBuilder(initialPoseRight)
                    .turnTo(ClimbHeading)
                    .strafeTo(new Vector2d(24, -10));

            Action ActionClimb = Climb.build();

            TrajectoryActionBuilder rightmax = drive.actionBuilder(initialPoseRight)
                    .strafeTo(new Vector2d(64,-63.25))
                    .waitSeconds(20)
                    .strafeTo(new Vector2d(60,-63.25));
            Action Actionrightmax = rightmax.build();










            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
            //Actions.runBlocking(claw.closeClaw());

            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            //armController.initArm(),
                            Actionrightmax
                    )
            );

        }
    }
