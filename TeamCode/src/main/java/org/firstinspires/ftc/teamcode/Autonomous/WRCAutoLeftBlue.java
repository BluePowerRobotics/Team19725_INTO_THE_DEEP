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

//import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@Config
@Autonomous(name = "Auto_2025_Left_Blue", group = "Autonomous")
public class WRCAutoLeftBlue extends LinearOpMode {
    GoBildaPinpointDriver odo;
    boolean ifblue = true;
    FindCandidate CVModule;
    SixServoArmEasyAction sixServoArmEasyAction;
    double Heading = Math.PI/2;

    Pose2d initialPoseLeft = new Pose2d(-32.86, -63.5, Heading);

        //public final GoBildaPinpointDriver odo;



        //@Override
        public void runOpMode() {
            AddTele step1  = new AddTele("step1", 1, telemetry);
            AddTele step2  = new AddTele("step2", 2, telemetry);
            AddTele step3  = new AddTele("step3", 3, telemetry);
            AddTele step4  = new AddTele("step4", 4, telemetry);
            AddTele step5  = new AddTele("step5", 5, telemetry);

            double IntakeHeading = 0;   //todo  why not Math.PI
            Vector2d IntakePos1 = new Vector2d(-32.86, -10);
            Vector2d IntakePos2 = new Vector2d(-30, -10);

            //todo
            //    调整出发点位置！！！
            sixServoArmEasyAction = new SixServoArmEasyAction(hardwareMap, telemetry, gamepad2);
            CVModule = new FindCandidate();
            CVModule.init(hardwareMap, telemetry, 0);

            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseLeft);

            //Arm armController = new Arm(hardwareMap,telemetry);
            //旋转角度的cos和sin的值（0，1）-> 90°

            TrajectoryActionBuilder Intake1 = drive.actionBuilder(initialPoseLeft)

                    .strafeTo(IntakePos1);
            Action ActionIntake1 = Intake1.build();

            TrajectoryActionBuilder Intake2 = Intake1.endTrajectory().fresh()
                    .turnTo(IntakeHeading)
                    .strafeTo(IntakePos2);
            Action ActionIntake2 = Intake2.build();







            // actions that need to happen on init; for instance, a claw tightening.
            //todo 初始化机器
           //armController1.initArm();
            Actions.runBlocking(
                    sixServoArmEasyAction.SixServoArmprepareForTaking()
            );

            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(

                    new SequentialAction(
                            ActionIntake1,
                            ActionIntake2
                    )
            );
            while(drive.localizer.getPose().position.y < 10){
                //ArmAction armAction = CVModule.CalculateAverage(CVModule);
                ArmAction armAction = CVModule.findCandidate();
                if(armAction.suggestion != -2){
                    if(armAction.suggestion == -1){
                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(0, 0.2),
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
//                        Actions.runBlocking(
//                                sixServoArmEasyAction.SixServoArmRunToPosition(armAction)
//                        );
                    }
                }
            }
        }
    }
