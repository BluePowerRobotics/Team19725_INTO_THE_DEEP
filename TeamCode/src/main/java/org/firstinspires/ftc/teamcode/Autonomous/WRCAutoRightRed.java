package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Installer.InstallerAction;
import org.firstinspires.ftc.teamcode.Controllers.OutPut.OutputAction;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyAction;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@Config
@Autonomous(name = "Auto_2025_Right_Red", group = "Autonomous")
public class WRCAutoRightRed extends LinearOpMode {
    public static double Dive_Y = -35;
    public static double ClipFinish_X = 62;

    public static double ClipStart_X = 45;
    public static double Output2_X = -4;
    public static  double Intake_X = 51.2;
    public static  double Intake_Y = -38;
    public static Pose2d EndPose = new Pose2d(0,0,0);
    public void runOpMode() {
        double startTime = System.currentTimeMillis();



        AddTele step1  = new AddTele("step1", 1, telemetry);
        AddTele step2  = new AddTele("step2", 2, telemetry);
        AddTele step3  = new AddTele("step3", 3, telemetry);
        AddTele step4  = new AddTele("step4", 4, telemetry);
        AddTele step5  = new AddTele("step5", 5, telemetry);


        double Heading = Math.toRadians(90);
        Pose2d initialPoseRight = new Pose2d(16.4, -63.5, Heading);

        Vector2d OutPutPos1 = new Vector2d(0,    Dive_Y);
        Vector2d OutPutBackPos = new Vector2d(0, Dive_Y - 2);
        Vector2d OutPutPos2 = new Vector2d(Output2_X,Dive_Y);
        Vector2d IntakePos = new Vector2d(Intake_X,Intake_Y);
        Vector2d ClipStartPos = new Vector2d(ClipStart_X,-63.5);
        Vector2d ClipFinishPos = new Vector2d(ClipFinish_X,-63.5);

        //todo
        //    调整出发点位置！！！

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseRight);
        OutputAction outputAction = new OutputAction(hardwareMap);
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
        SixServoArmEasyAction sixServoArmAction = new SixServoArmEasyAction(hardwareMap, telemetry,gamepad2);
        ServoValueEasyOutputter servoValueEasyOutputter = sixServoArmAction.servoValueOutputter;
        InstallerAction installerAction = new InstallerAction(hardwareMap, gamepad1, gamepad2, telemetry);



        // actions that need to happen on init; for instance, a claw tightening.
        //todo 初始化机器
        CVModule.init(hardwareMap, telemetry, 1);
            Actions.runBlocking(
                    new ParallelAction(
                            sixServoArmAction.SixServoArmInit(),
                            outputAction.OutPutArmDown()
                    )


            );

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        outputAction.OutPutArmMid(),
                        ActionOutPut1,
                        outputAction.OutPutArmUp(),

                        //output Action
                        //ActionOutPutBack,
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose()).lineToY(-40).build(),
                                outputAction.setClip(false),
                                drive.actionBuilder(drive.localizer.getPose()).lineToY(-35).build()
                        ),
                        step1.addTele(),
                        ActionIntake1,
                        sixServoArmAction.SixServoArmRunToPosition(CVModule.CalculateAverage(CVModule)),


                        step2.addTele(),
                        new ParallelAction(
                                new SequentialAction(
                                        ActionGetClip,
                                        ActionGetClipFinish
                                ),
                                sixServoArmAction.SixServoArmGiveTheSample(),
                                installerAction.installerPuller()
                        ),

                        step3.addTele(),



                        outputAction.OutPutArmDown(),
                        outputAction.setClip(true),
                        installerAction.clipInstaller(true),

                        step4.addTele(),
                        //sixServoArmAction.
                        installerAction.spitClip(),
                        outputAction.OutPutArmUp(),
                        step5.addTele(),
                        ActionOutput2,
                        outputAction.OutPutArmDown(),
                        new SequentialAction(
                                drive.actionBuilder(drive.localizer.getPose()).lineToY(-40).build(),
                                outputAction.setClip(false),
                                drive.actionBuilder(drive.localizer.getPose()).lineToY(-35).build()
                        )
                )
        );




        //第一场注释掉
        while(drive.localizer.getPose().position.x < 7){
            ArmAction armAction = CVModule.CalculateAverage(CVModule);
            if(armAction.suggestion != -2){
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
                            sixServoArmAction.SixServoArmRunToPosition(armAction)
                    );
                }
            }
        }
        if(startTime - System.currentTimeMillis() > 28000){
            servoValueEasyOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
        }
        EndPose = drive.localizer.getPose();
    }
    }
