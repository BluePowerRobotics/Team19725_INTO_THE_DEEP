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

import org.firstinspires.ftc.teamcode.OpModes.ArmController;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.ArmController;

@Config
@Autonomous(name = "Auto_2025_Left", group = "Autonomous")
public class Auto2025Left extends LinearOpMode {
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

            Pose2d initialPoseLeft = new Pose2d(-34, -64.575, Math.toRadians(90.00));



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

            //todo
            //    调整出发点位置！！！

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseLeft);

            Arm armController = new Arm(hardwareMap,telemetry);
            AutoAlignment autofollow = new AutoAlignment(hardwareMap, drive, telemetry,ifblue);
            //旋转角度的cos和sin的值（0，1）-> 90°



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




            ArmController armController1 = new ArmController();








            TrajectoryActionBuilder climb2 = drive.actionBuilder(new Pose2d(-33.89, -66.29, Math.toRadians(90.00)))
                    .splineTo(new Vector2d(-33.89, -10.38), Math.toRadians(90.00))
                    .splineTo(new Vector2d(-23.39, -0.34), Math.toRadians(90.00));
            Action CloseOutclimb2 = climb2.endTrajectory().fresh()
                    .build();
            Action Actionclimb2 = climb2.build();




            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
            armController1.initArm(hardwareMap, gamepad1, gamepad2, telemetry);

            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(

                    new SequentialAction(
                            Actionclimb2,
                            CloseOutclimb2
                    )
            );

        }
    }
