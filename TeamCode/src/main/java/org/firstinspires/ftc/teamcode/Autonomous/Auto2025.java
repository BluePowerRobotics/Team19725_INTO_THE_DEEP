package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
@Autonomous(name = "Auto_2025", group = "Autonomous")
public class Auto2025 extends LinearOpMode {
    GoBildaPinpointDriver odo;
    boolean ifblue = true;



        //public final GoBildaPinpointDriver odo;



        //@Override
        public void runOpMode() {
            Pose2d initialPoseLeft = new Pose2d(-24, -64.575, Math.toRadians(180.00));
            Pose2d initialPose = initialPoseLeft;

            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            Arm armController = new Arm(hardwareMap,telemetry);
            AutoAlignment autofollow = new AutoAlignment(hardwareMap, drive, telemetry,ifblue);
            //旋转角度的cos和sin的值（0，1）-> 90°

            TrajectoryActionBuilder Basket1 = drive.actionBuilder(new Pose2d(-23.62, -64.58, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(-56.25, -57.85), Math.toRadians(135.00));
            Action CloseOutBasket1 = Basket1.endTrajectory().fresh()
                    .build();
            Action ActionBasket1 = Basket1.build();




            TrajectoryActionBuilder Clip1 = drive.actionBuilder(new Pose2d(-60.26, -59.36, Math.toRadians(135.00)))
                    .splineTo(new Vector2d(-58.91, -37.92), Math.toRadians(180.00));
            Action CloseOutClip1 = Clip1.endTrajectory().fresh()
                    .build();
            Action ActionClip1 = Clip1.build();



            TrajectoryActionBuilder Basket2 = drive.actionBuilder(new Pose2d(-60.04, -35.89, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(-53.72, -53.27), Math.toRadians(248.20))
                    .splineTo(new Vector2d(-58.46, -57.33), Math.toRadians(135.00));
            Action CloseOutBasket2 = Basket2.endTrajectory().fresh()
                    .build();
            Action ActionBasket2 = Basket2.build();














            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
            //Actions.runBlocking(claw.closeClaw());



            telemetry.addData("Starting Position", initialPose.toString());
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            armController.initArm(),
                            new ParallelAction(
                                    new SequentialAction(
                                            ActionBasket1,
                                            CloseOutBasket1
                                    ),
                                    armController.outPut()
                            ),
                            new SequentialAction(
                                ActionClip1,
                                CloseOutClip1
                            ),
                            new ParallelAction(
                                    autofollow.follow(),
                                    armController.inTake(20, -10)
                            ),
                            new ParallelAction(
                                new SequentialAction(
                                    ActionBasket2,
                                    CloseOutBasket2
                            ),
                            armController.outPut()
                        )
                    )
            );


//            Actions.runBlocking(
//                new SequentialAction(
//
//                        armController.initArm(),
//
//                        new SequentialAction(
//                                Actioncliptest,
//
//                                CloseOutcliptest
//                        ),
//                        armController.inTake(560,0)
//
////                    new SequentialAction(
////                            Actiontest2,
////                            CloseOuttest2
////                    )
//                )
//            );


//            Actions.runBlocking(
//                    new SequentialAction(
//                            armController.initArm(),
//                            armController.inTake(300, 0.5),
//                            armController.outPut()
//                    )
//            );
        }
    }
