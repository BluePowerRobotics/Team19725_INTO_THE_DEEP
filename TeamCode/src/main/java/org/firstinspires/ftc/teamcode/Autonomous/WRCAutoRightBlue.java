package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@Config
@Autonomous(name = "Auto_2025_Right_Blue", group = "Autonomous")
public class WRCAutoRightBlue extends LinearOpMode {
        public static double Dive_Y = -35;
        public static double ClipFinish_X = 62;

        public static double ClipStart_X = 45;
        public static double Output1_X = 4;
        public static  double Intake_X = 51.2;
        public static  double Intake_Y = -38;
        public static Pose2d EndPose = new Pose2d(0,0,0);
        public void runOpMode() {
            AddTele step1  = new AddTele("step1", 1, telemetry);
            AddTele step2  = new AddTele("step2", 2, telemetry);
            AddTele step3  = new AddTele("step3", 3, telemetry);
            AddTele step4  = new AddTele("step4", 4, telemetry);
            AddTele step5  = new AddTele("step5", 5, telemetry);


            double Heading = Math.toRadians(90);
            Pose2d initialPoseRight = new Pose2d(16.4, -63.5, Heading);

            Vector2d OutPutPos1 = new Vector2d(0,    Dive_Y);
            Vector2d OutPutBackPos = new Vector2d(0, Dive_Y - 2);
            Vector2d OutPutPos2 = new Vector2d(Output1_X,Dive_Y);
            Vector2d IntakePos = new Vector2d(Intake_X,Intake_Y);
            Vector2d ClipStartPos = new Vector2d(ClipStart_X,-63.5);
            Vector2d ClipFinishPos = new Vector2d(ClipFinish_X,-63.5);

            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseRight);
            //旋转角度的cos和sin的值（0，1）-> 90°



            TrajectoryActionBuilder OutPut1 = drive.actionBuilder(initialPoseRight)
                    .strafeTo(OutPutPos1)
                    .waitSeconds(2)
                    ;

            Action ActionOutPut1 = OutPut1.build();

            TrajectoryActionBuilder OutPutBack = OutPut1.endTrajectory().fresh()
                    .strafeTo(OutPutBackPos)
                    .waitSeconds(2)
                    ;

            Action ActionOutPutBack = OutPutBack.build();

            TrajectoryActionBuilder Intake1 = OutPutBack.endTrajectory().fresh()
                    .strafeTo(IntakePos)
                    .waitSeconds(2)
                    ;
            Action ActionIntake1 = Intake1.build();

            TrajectoryActionBuilder GetClip = Intake1.endTrajectory().fresh()
                    .strafeTo(ClipStartPos)
                    .waitSeconds(2)
                    ;
            Action ActionGetClip = GetClip.build();

            TrajectoryActionBuilder GetClipFinish = GetClip.endTrajectory().fresh()
                    .strafeTo(ClipFinishPos)
                    .waitSeconds(2)
                    ;
            Action ActionGetClipFinish = GetClipFinish.build();

            TrajectoryActionBuilder Output2 = GetClipFinish.endTrajectory().fresh()
                    .strafeTo(OutPutPos2)
                    .waitSeconds(2)
                    ;
            Action ActionOutput2 = Output2.build();

            FindCandidate CVModule = new FindCandidate();
            SixServoArmEasyAction sixServoArmController = new SixServoArmEasyAction(hardwareMap, telemetry,gamepad2);




            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
            CVModule.init(hardwareMap, telemetry, 0);
//            Actions.runBlocking(
//                    sixServoArmController.SixServoArmInit()
//            );

            waitForStart();

            if (isStopRequested()) return;
            Actions.runBlocking(
                    new SequentialAction(
                            ActionOutPut1,
                            //output Action
                            //ActionOutPutBack,
                            new SequentialAction(
                                    drive.actionBuilder(drive.localizer.getPose()).lineToY(-40).build(),
                                    drive.actionBuilder(drive.localizer.getPose()).lineToY(-35).build()
                            ),
                            ActionIntake1,
                            ActionGetClip,
                            ActionGetClipFinish,
                            //install action
                            ActionOutput2
                    )
            );
            while(drive.localizer.getPose().position.x > -7){
                ArmAction armAction = CVModule.findCandidate();
                if(armAction.suggestion == -1){
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(0, -0.2),
                                    0
                            )
                    );
                }
                else{
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(0, 0),
                                    0
                            )
                    );
                    Actions.runBlocking(
                            sixServoArmController.SixServoArmRunToPosition(armAction)
                    );
                }
            }
            EndPose = drive.localizer.getPose();
        }
    }
