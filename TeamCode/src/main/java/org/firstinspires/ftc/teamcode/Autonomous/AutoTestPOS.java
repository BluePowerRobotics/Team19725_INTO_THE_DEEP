package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

@Config
@Disabled
public class AutoTestPOS extends LinearOpMode {
    GoBildaPinpointDriver odo;
    boolean ifblue = true;





        //public final GoBildaPinpointDriver odo;



        //@Override
        public void runOpMode() {
            Pose2d initialPoseLeft = new Pose2d(-24, -48, Math.toRadians(180.00));
            Pose2d initialPosetest1 = new Pose2d(-24.15, -62.75, Math.toRadians(90.00));
            Pose2d initialPosetmp = new Pose2d(0.23, -60.26, Math.toRadians(90.00));
            Pose2d initialPosetest2 = new Pose2d(-24.00, -48.00, Math.toRadians(90.00));



            double OutPutHeading = Math.toRadians(135);
            double IntakeHeading = 3.1415926;   //todo  why not Math.PI
            double Intake3Heading = Math.toRadians(228.81);
            Vector2d OutPutPos = new Vector2d(-53.9,-53.9);
            Pose2d OutPutFinishPos = new Pose2d(-53.9,-53.9, OutPutHeading);
            Vector2d IntakePos1 = new Vector2d(-49.18,-43.67);
            Pose2d IntakeFinish1 = new Pose2d(-49.18,-43.67, IntakeHeading);
            Vector2d IntakePos2 = new Vector2d(-60.22,-43.67);
            Pose2d IntakeFinish2 = new Pose2d(-60.22,-43.67, IntakeHeading);
            Vector2d IntakePos3 = new Vector2d(-60,-44);
            //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseLeft);

            //telemetry.addData("设置tele", 1);
            //drive.settele(telemetry);
            //Claw claw = new Claw(hardwareMap);
            //Arm armController = new Arm(hardwareMap,telemetry);
            //AutoAlignment autofollow = new AutoAlignment(hardwareMap, drive, telemetry,ifblue);


            AddTele step1  = new AddTele("step1", 1, telemetry);
            AddTele step2  = new AddTele("step2", 2, telemetry);
            AddTele step3  = new AddTele("step3", 3, telemetry);
            AddTele step4  = new AddTele("step4", 4, telemetry);
            AddTele step5  = new AddTele("step5", 5, telemetry);






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


            TrajectoryActionBuilder straightclip = drive.actionBuilder(new Pose2d(0,0,0))

                    .waitSeconds(0.5)
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(0.5)
                    .strafeTo(OutPutPos)

                    .waitSeconds(0.5)
                    .turnTo(Math.PI)
                    .strafeToConstantHeading(new Vector2d(-48,-30))
                    ;

            Action CloseOutstraightclip = straightclip.endTrajectory().fresh()
                    .build();
            Action Actionstraightclip = straightclip.build();



            //todo set initPOS

            TrajectoryActionBuilder OutPut0 = drive.actionBuilder(initialPoseLeft)
                    .turnTo(OutPutHeading)
                    .strafeTo(OutPutPos);

            Action ActionOutPut0 = OutPut0.build();

            TrajectoryActionBuilder Intake1 = OutPut0.endTrajectory().fresh()
                    .turnTo(IntakeHeading)
                    .strafeTo(IntakePos1);
            Action ActionIntake1 = Intake1.build();

            TrajectoryActionBuilder OutPut1 = Intake1.endTrajectory().fresh()
                    .turnTo(OutPutHeading)
                    .strafeTo(OutPutPos);

            Action ActionOutPut1 = OutPut1.build();

            TrajectoryActionBuilder Intake2 = OutPut1.endTrajectory().fresh()
                    .turnTo(IntakeHeading)
                    .strafeTo(IntakePos2);
            Action ActionIntake2 = Intake2.build();

            TrajectoryActionBuilder OutPut2 = Intake2.endTrajectory().fresh()
                    .turnTo(OutPutHeading)
                    .strafeTo(OutPutPos);

            Action ActionOutPut2 = OutPut2.build();








            TrajectoryActionBuilder Intake3 = drive.actionBuilder(drive.localizer.getPose())
                    .turnTo(Intake3Heading)
                    .strafeTo(IntakePos3);
            Action CloseOutIntake3 = Intake3.endTrajectory().fresh()
                    .build();
            Action ActionIntake3 = Intake3.build();

            //setreversed

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
                            //armController.initArm(),

                            ActionOutPut0,

                            //armController.outPut(),

                            ActionIntake1,

                            //armController.inTake(200, 0),

                            ActionOutPut1,

                            //armController.outPut(),

                            ActionIntake2,

                            //armController.inTake(200, 0),

                            ActionOutPut2

                            //armController.outPut()
                    )
            );
        }
    }
