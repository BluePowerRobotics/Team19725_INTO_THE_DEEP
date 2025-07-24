package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.WRCAutoRightBlue;
import org.firstinspires.ftc.teamcode.Controllers.IntakeLength.MotorLineIntakeLengthController;
import org.firstinspires.ftc.teamcode.Controllers.OutPut.OutputController;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyController;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@Config
@TeleOp
public class OP0723 extends LinearOpMode {
    MecanumDrive drive;
    SixServoArmEasyController sixServoArmEasyController;
    ServoValueEasyOutputter servoValueOutputter;
    OutputController outputController;
    MotorLineIntakeLengthController intakeLengthController;
    FindCandidate CVModule;
    int FrameCnt = 0;
    long t;
    ArmAction Sum = new ArmAction(0,0,0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        t = System.currentTimeMillis();
        initHardware();
        waitForStart();
        while(opModeIsActive()){
            teleprint();
            controlChassis();
            outputControl();
            intakeAndOutputControl();
            intakeLengthControl();
            intakeAndOutputControl_2();
            intakeLengthControl_2();
        }
    }
    public double intakeStartLength;
    public static double intakeLengthAdd=147;
    public void initHardware(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //todo 读取自动化结束位置
        drive = new MecanumDrive(hardwareMap, WRCAutoRightBlue.EndPose);
        //todo 0位初始化&定点移动
        intakeLengthController = new MotorLineIntakeLengthController(hardwareMap);
        sixServoArmEasyController = SixServoArmEasyController.getInstance(hardwareMap, telemetry);
        servoValueOutputter = sixServoArmEasyController.servoValueOutputter;
        outputController = new OutputController(hardwareMap);
        CVModule = new FindCandidate();
        //todo: 这里的颜色需要根据实际情况调整!!!!!!!
        CVModule.init(hardwareMap, telemetry, 0);
        //Actions.runBlocking(sixServoArmEasyController.SixServoArmInit());
        double t = System.currentTimeMillis(); // 获取当前时间
        //硬件初始化
        servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
        sixServoArmEasyController.inToTheDeep();
        intakeStartLength=intakeLengthController.getIntakeLengthCurrentPosition();
    }
    public static double kpad = 1;
    public void controlChassis(){
        double realx = -(gamepad1.left_stick_y+gamepad2.left_stick_y) * kpad;
        double realy = -(gamepad1.left_stick_x+gamepad2.left_stick_x) * kpad;//*****已经转换过X,Y轴，并写过负号
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        realx,
                        realy
                ),
                -(gamepad1.right_stick_x+gamepad2.right_stick_x) * kpad
        ));
    }
    void teleprint(){
        sleep(1);
        telemetry.addData("simpleFPS", 1000/(System.currentTimeMillis() - t));
        t = System.currentTimeMillis();

        telemetry.addData("intakeLength",intakeLengthController.getIntakeLengthCurrentPosition());
        telemetry.addData("xpad(实际上是y轴)", gamepad1.left_stick_y * kpad);
        telemetry.addData("ypad(实际上是x轴)", gamepad1.left_stick_x * kpad);
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    boolean gp1Xpressed=false;
    void outputControl(){
//        if(gamepad1.x){
//            if(!gp1Xpressed){
//                outputController.setMode(outputController.outputStates.next());
//                gp1Xpressed=true;
//            }
//        }else{
//            gp1Xpressed=false;
//        }
    }
    long startT = 0;
    boolean ifinited = false;
    boolean HasSetArmPos=false;
    boolean gp1FirstPressed =false;
    boolean gp1FirstReleased =false;
    boolean gp1SecondPressed =false;
    void intakeAndOutputControl(){
        if(gamepad1.dpad_left){
            gp1FirstPressed =true;
            if(!gp1FirstReleased){
                sixServoArmEasyController.inToTheDeep();
                outputController.ArmMiddle();
            }else{
                sixServoArmEasyController.scanTheSample();
                outputController.ArmMiddle();
                HasSetArmPos=false;
            }
        }else{
            sixServoArmEasyController.PFTinited = false;
            if(gp1FirstPressed) gp1FirstReleased =true;
            if(gp1SecondPressed) {
                gp1FirstReleased = false;
                gp1SecondPressed = false;
                gp1FirstPressed = false;
            }
        }

        //todo 清除空值
        if(gamepad1.dpad_up){
            //ArmAction Command = CVModule.CalculateAverage(CVModule);
            ArmAction Command = CVModule.findCandidate();
            if(Command.suggestion != -2 && !HasSetArmPos){
                sixServoArmEasyController.setTargetPosition(Command).update();
                HasSetArmPos = true;
            }
        }
        if(gamepad1.dpad_right){
            if(!ifinited){
                HasSetArmPos = false;
                ifinited = true;
                startT = System.currentTimeMillis();
                servoValueOutputter.SingleServoControl(1, servoValueOutputter.getServoPosition(1) - 0.1);
            }

            if(System.currentTimeMillis() - startT > 100){
                servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
            }
            if(System.currentTimeMillis()-startT>400){
                servoValueOutputter.DegreeServoControl(0,90);
                servoValueOutputter.DegreeServoControl(1,45);
                servoValueOutputter.DegreeServoControl(2,135);
                servoValueOutputter.DegreeServoControl(3,90);
                servoValueOutputter.DegreeServoControl(4,0);
            }
        }
        else{
            ifinited = false;
        }
        if(gamepad1.dpad_down){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
        }

        if(gamepad1.x){
            sixServoArmEasyController.eatHuman();
        }
        if(gamepad1.y){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
            outputController.ArmDown();
            outputController.setClip(false);
        }
        if(gamepad1.b){
            sixServoArmEasyController.giveTheSample();
        }
        if(gamepad1.a){
            outputController.setClip(true);
        }
        if(gamepad1.left_bumper){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
            outputController.ArmUp();
        }
        if(gamepad1.right_bumper){
            outputController.setClip(false);
            outputController.ArmDown();
        }
    }
    boolean lengthAdded=false;
    void intakeLengthControl(){
        if(gamepad1.x){
            if(!lengthAdded){
                intakeLengthController.setIntakeTargetPosition(intakeStartLength+intakeLengthAdd);
                lengthAdded=true;
            }
        }
        if(lengthAdded)
            intakeLengthController.update();
        if(gamepad1.left_trigger > 0.1){
            intakeLengthController.SingleMotorControl(gamepad1.left_trigger);//放出滑轨
        }
        if(gamepad1.right_trigger > 0.1){
            intakeLengthController.SingleMotorControl(-gamepad1.right_trigger);//收回滑轨
        }
    }
    boolean gp2FirstPressed=false;
    boolean gp2FirstReleased=false;
    boolean gp2SecondPressed=false;
    void intakeAndOutputControl_2(){
        if(gamepad2.dpad_left){
            gp2FirstPressed=true;
            if(!gp2FirstReleased){
                sixServoArmEasyController.inToTheDeep();
                outputController.ArmMiddle();
            }else{
                sixServoArmEasyController.scanTheSample();
                outputController.ArmMiddle();
                HasSetArmPos=false;
            }
        }else{
            sixServoArmEasyController.PFTinited = false;
            if(gp2FirstPressed) gp2FirstReleased=true;
            if(gp2SecondPressed) {
                gp2FirstReleased = false;
                gp2SecondPressed = false;
                gp2FirstPressed = false;
            }
        }

        //todo 清除空值
        if(gamepad2.dpad_up){
            //ArmAction Command = CVModule.CalculateAverage(CVModule);
            ArmAction Command = CVModule.findCandidate();
            if(Command.suggestion != -2 && !HasSetArmPos){
                sixServoArmEasyController.setTargetPosition(Command).update();
                HasSetArmPos = true;
            }
        }
        if(gamepad2.dpad_right){
            if(!ifinited){
                HasSetArmPos = false;
                ifinited = true;
                startT = System.currentTimeMillis();
                servoValueOutputter.SingleServoControl(1, servoValueOutputter.getServoPosition(1) - 0.1);
            }

            if(System.currentTimeMillis() - startT > 100){
                servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
            }
            if(System.currentTimeMillis()-startT>400){
                servoValueOutputter.DegreeServoControl(0,90);
                servoValueOutputter.DegreeServoControl(1,45);
                servoValueOutputter.DegreeServoControl(2,135);
                servoValueOutputter.DegreeServoControl(3,90);
                servoValueOutputter.DegreeServoControl(4,0);
            }
        }
        else{
            ifinited = false;
        }
        if(gamepad2.dpad_down){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
        }

        if(gamepad2.x){
            sixServoArmEasyController.eatHuman();
        }
        if(gamepad2.y){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.LOCKED);
            outputController.ArmDown();
            outputController.setClip(false);
        }
        if(gamepad2.b){
            sixServoArmEasyController.giveTheSample();
        }
        if(gamepad2.a){
            outputController.setClip(true);
        }
        if(gamepad2.left_bumper){
            servoValueOutputter.setClip(ServoValueEasyOutputter.ClipPosition.UNLOCKED);
            outputController.ArmUp();
        }
        if(gamepad2.right_bumper){
            outputController.setClip(false);
            outputController.ArmDown();
        }
    }
    void intakeLengthControl_2(){
        if(gamepad2.x){
            if(!lengthAdded){
                intakeLengthController.setIntakeTargetPosition(intakeStartLength+intakeLengthAdd);
                lengthAdded=true;
            }
        }
        if(lengthAdded)
            intakeLengthController.update();
        if(gamepad2.left_trigger > 0.1){
            intakeLengthController.SingleMotorControl(gamepad2.left_trigger);//放出滑轨
        }
        if(gamepad2.right_trigger > 0.1){
            intakeLengthController.SingleMotorControl(-gamepad2.right_trigger);//收回滑轨
        }
    }
}
