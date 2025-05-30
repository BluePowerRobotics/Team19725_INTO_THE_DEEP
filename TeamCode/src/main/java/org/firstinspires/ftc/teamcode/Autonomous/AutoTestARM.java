package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Disabled
public class AutoTestARM extends LinearOpMode {
    GoBildaPinpointDriver odo;
    boolean ifblue = true;


        public class addTele implements Action {
            private boolean initialized = false;
            //telemetry = new MultipleTelemetry(telemetry);
            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    telemetry = new MultipleTelemetry(telemetry);
                    initialized = true;
                }

                telemetry.addData("xpod", odo.getEncoderX());
                telemetry.addData("ypod", odo.getEncoderY());
                telemetry.update();
                return false;
            }


        }


        //public final GoBildaPinpointDriver odo;



        //@Override
        public void runOpMode() {
            Pose2d initialPoseLeft = new Pose2d(-24.15, -62.75, Math.toRadians(90.00));
            Pose2d initialPosetest1 = new Pose2d(-24.15, -62.75, Math.toRadians(90.00));
            Pose2d initialPosetmp = new Pose2d(0.23, -60.26, Math.toRadians(90.00));
            Pose2d initialPosetest2 = new Pose2d(-24.00, -48.00, Math.toRadians(90.00));
            //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPosetmp);

            //telemetry.addData("设置tele", 1);
            //drive.settele(telemetry);
            //Claw claw = new Claw(hardwareMap);
            Arm armController = new Arm(hardwareMap,telemetry);
            //AutoAlignment autofollow = new AutoAlignment(hardwareMap, drive, telemetry,ifblue);


            TrajectoryActionBuilder clip = drive.actionBuilder(new Pose2d(-23.85, -48.04, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(-48.04, -34.80), Math.toRadians(180.75));
            Action CloseOutclip = clip.endTrajectory().fresh()
                    .build();
            Action Actionclip = clip.build();


            //test1
            TrajectoryActionBuilder test1 = drive.actionBuilder(new Pose2d(-48.00, -24.00, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(-48.00, 24.00), Math.toRadians(90));
            Action CloseOuttest1 = test1.endTrajectory().fresh()
                    .build();
            Action Actiontest1 = test1.build();


            //test2
            TrajectoryActionBuilder test2 = drive.actionBuilder(new Pose2d(-24.00, -48.00, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(0.11, -37.31), Math.toRadians(0.00))
                    .splineTo(new Vector2d(24.00, -48.00), Math.toRadians(270.00));
            Action CloseOuttest2 = test2.endTrajectory().fresh()
                    .build();
            Action Actiontest2 = test2.build();


            //绕场一周
            TrajectoryActionBuilder Tapround = drive.actionBuilder(new Pose2d(-24.15, -62.75, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(-39.05, -54.17), Math.toRadians(90.00))
                    .splineTo(new Vector2d(-39.95, 23.70), Math.toRadians(90.00))
                    .splineTo(new Vector2d(-0.68, 39.27), Math.toRadians(0.00))
                    .splineTo(new Vector2d(38.14, 30.47), Math.toRadians(270.00))
                    .splineTo(new Vector2d(37.92, -48.53), Math.toRadians(269.84))
                    .splineTo(new Vector2d(14.45, -55.52), Math.toRadians(173.22))
                    .splineTo(new Vector2d(-24.60, -55.52), Math.toRadians(187.47));
            Action CloseOutTapround = Tapround.endTrajectory().fresh()
                    .build();
            Action ActionTapround = Tapround.build();


            TrajectoryActionBuilder Round2 = drive.actionBuilder(new Pose2d(-24.15, -48.08, Math.toRadians(180.00)))
                    .splineTo(new Vector2d(-48.08, -23.92), Math.toRadians(90.63))
                    .splineTo(new Vector2d(-47.85, 23.92), Math.toRadians(89.73))
                    .splineTo(new Vector2d(-23.70, 47.85), Math.toRadians(0.00))
                    .splineTo(new Vector2d(23.70, 47.40), Math.toRadians(-0.55))
                    .splineTo(new Vector2d(48.08, 23.70), Math.toRadians(-88.94))
                    .splineTo(new Vector2d(48.08, -24.60), Math.toRadians(270.00))
                    .splineTo(new Vector2d(23.70, -48.08), Math.toRadians(179.53))
                    .splineTo(new Vector2d(-24.15, -48.30), Math.toRadians(181.51));
            Action CloseOutRound2 = Round2.endTrajectory().fresh()
                    .build();
            Action ActionRound2 = Round2.build();


//            new SequentialAction(
//                    ActionRound2,
//                    CloseOutRound2
//            )

//            new SequentialAction(
//                    ActionTapround,
//                    CloseOutTapround
//            )

            TrajectoryActionBuilder cliptest = drive.actionBuilder(new Pose2d(0.23, -60.26, Math.toRadians(90.00)))
                    .waitSeconds(1.0)
                    .splineTo(new Vector2d(-48.08, -36.11), Math.toRadians(180.00))
                    .splineTo(new Vector2d(-38.60, -55.52), Math.toRadians(226.47))
                    .splineTo(new Vector2d(-57.78, -56.88), Math.toRadians(141.34))
                    .waitSeconds(1.0);
            Action CloseOutcliptest = cliptest.endTrajectory().fresh()
                    .build();
            Action Actioncliptest = cliptest.build();



            TrajectoryActionBuilder tap5 = drive.actionBuilder(new Pose2d(-5.13, -70.40, Math.toRadians(90.00)))
                    .splineToSplineHeading(new Pose2d(-35.94, -24.08, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .splineTo(new Vector2d(-47.17, -53.27), Math.toRadians(248.96))
                    .splineToSplineHeading(new Pose2d(-59.13, -58.68, Math.toRadians(225.00)), Math.toRadians(225.00));
            Action trajectoryActionCloseOuttap5 = tap5.endTrajectory().fresh()
                    .build();
            Action trajectoryActionChosentap5 = tap5.build();

            // vision here that outputs position
            int visionOutputPosition = 4;
            TrajectoryActionBuilder test = drive.actionBuilder(initialPoseLeft)
                    .splineToSplineHeading(new Pose2d(-42.66, -18.73, Math.toRadians(180.00)), Math.toRadians(114.92));

            Action trajectoryActionCloseOutTest = test.endTrajectory().fresh()
                    .build();


            //旋转角度的cos和sin的值（0，1）-> 90°
            //trajectory0.build();
//                .build();

//                .lineToY(48)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);



            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
            //Actions.runBlocking(claw.closeClaw());


            while (!isStopRequested() && !opModeIsActive()) {
                int position = visionOutputPosition;
                telemetry.addData("Position during Init", "test1");
                telemetry.update();
            }

            int startPosition = visionOutputPosition;
            telemetry.addData("Starting Position", "test1");
            telemetry.update();
            waitForStart();

            if (isStopRequested()) return;

//            Actions.runBlocking(
//                    new SequentialAction(
//                            autofollow.follow()
//                    )
//            );


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


            Actions.runBlocking(
                    new SequentialAction(
                            armController.initArm(),
                            //armController.inTake(300, 0.5),
                            armController.outPut()
                    )
            );
        }
    }
