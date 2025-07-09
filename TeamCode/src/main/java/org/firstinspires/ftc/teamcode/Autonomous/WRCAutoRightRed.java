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

import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArmAction;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.VisualColor.FindCandidate;
import org.firstinspires.ftc.teamcode.VisualColor.model.ArmAction;

@Config
@Autonomous(name = "Auto_2025_Right_Red", group = "Autonomous")
public class WRCAutoRightRed extends LinearOpMode {
    public void runOpMode() {
        AddTele step1  = new AddTele("step1", 1, telemetry);
        AddTele step2  = new AddTele("step2", 2, telemetry);
        AddTele step3  = new AddTele("step3", 3, telemetry);
        AddTele step4  = new AddTele("step4", 4, telemetry);
        AddTele step5  = new AddTele("step5", 5, telemetry);


        double Heading = Math.toRadians(90);
        Pose2d initialPoseRight = new Pose2d(16.4, -63.5, Heading);




        Vector2d OutPutPos0 = new Vector2d(0,-32.5);
        Pose2d OutPut0FinishPos = new Pose2d(6,-32.5, Heading);
        Vector2d OutPutPos1 = new Vector2d(6,-32.5);
        Pose2d OutPut1FinishPos = new Pose2d(6,-32.5, Heading);
        Vector2d IntakePos = new Vector2d(49,-40);
        Pose2d IntakeFinish = new Pose2d(49,-40, Heading);
        Vector2d ClipPos = new Vector2d(49,-63.5);
        Pose2d ClipFinish = new Pose2d(49,-63.5, Heading);

        //todo
        //    调整出发点位置！！！

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseRight);
        //旋转角度的cos和sin的值（0，1）-> 90°



        TrajectoryActionBuilder OutPut0 = drive.actionBuilder(initialPoseRight)
                .strafeTo(OutPutPos0);

        Action ActionOutPut0 = OutPut0.build();

        TrajectoryActionBuilder Intake1 = OutPut0.endTrajectory().fresh()
                .strafeTo(IntakePos);
        Action ActionIntake1 = Intake1.build();

        TrajectoryActionBuilder GetClip = Intake1.endTrajectory().fresh()
                .strafeTo(ClipPos);

        Action ActionGetClip = GetClip.build();

        TrajectoryActionBuilder Output1 = GetClip.endTrajectory().fresh()
                .strafeTo(OutPutPos1);
        Action ActionOutput1 = Output1.build();

        FindCandidate CVModule = new FindCandidate();
        SixServoArmAction sixServoArmController = new SixServoArmAction(hardwareMap, telemetry);
        CVModule.init(hardwareMap, telemetry, 0);

        TrajectoryActionBuilder climb2 = drive.actionBuilder(new Pose2d(-33.89, -66.29, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-33.89, -10.38), Math.toRadians(90.00))
                .splineTo(new Vector2d(-23.39, -0.34), Math.toRadians(90.00));
        Action CloseOutclimb2 = climb2.endTrajectory().fresh()
                .build();
        Action Actionclimb2 = climb2.build();




        // actions that need to happen on init; for instance, a claw tightening.
        //todo 初始化机器
        CVModule.init(hardwareMap, telemetry, 1);
        Actions.runBlocking(
                sixServoArmController.SixServoArmInit()
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        ActionOutPut0,
                        //output Action
                        ActionIntake1,
                        ActionGetClip,
                        ActionOutput1
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

        }
    }
