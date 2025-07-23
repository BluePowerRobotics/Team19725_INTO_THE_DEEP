package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Controllers.ChassisController;
import org.firstinspires.ftc.teamcode.Controllers.Installer.InstallerController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeLength.*;
import org.firstinspires.ftc.teamcode.Controllers.OutPut.OutputController;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.ServoValueEasyOutputter;
import org.firstinspires.ftc.teamcode.Controllers.SixServoArm.SixServoArmEasyController;
import org.firstinspires.ftc.teamcode.Vision.FindCandidate;
import org.firstinspires.ftc.teamcode.Vision.model.ArmAction;

@TeleOp
@Config
public class OP07142 extends LinearOpMode{
    MotorLineIntakeLengthController intakeLengthController;
    ServoValueEasyOutputter servoValueEasyOutputter;
    SixServoArmEasyController sixServoArmController;
    //ChassisController rbmove = new ChassisController();// 构建Move_GYW（）class实例
    static DcMotor armPuller;
    static DcMotor leftFront, leftBack, rightBack, rightFront,intake,output;
    Servo servos3, servos4, servos5;


    IMU imu;
    int logoFacingDirectionPosition = 0;
    int usbFacingDirectionPosition = 2;
    boolean orientationIsValid = true;
    YawPitchRollAngles orientation;
    boolean HasSetArmPos = false;
    public double angletime = 0;

    public double t = 0;// 当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;
    public double move_x2;
    public double degree = 0;
    public double thita = 0;
    public double nuleftFront = 0.7071068;// 二分之根号二

    public boolean noheadmode = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;

    public boolean ahasbeenpressed = false, lbhasbeenpressed = false, rbhasbeenpressed = false;
    //public int servo_select = 3;
    //public double servo_position = 0.5;
    // servos5:0.86~1
    //ArmDirectController armDirectController = new ArmDirectController();
    ChassisController chassisController = new ChassisController();
    FindCandidate CVModule = new FindCandidate();
    //EasyClimb easyClimb = new EasyClimb();

    private void inithardware() {

        CVModule.init(hardwareMap, telemetry , 0);
        outputController = new OutputController(hardwareMap);
        installerController = new InstallerController(hardwareMap,gamepad1,gamepad2,telemetry);
        intakeLengthController = MotorLineIntakeLengthController.getInstance(hardwareMap);
        sixServoArmController=SixServoArmEasyController.getInstance(hardwareMap,telemetry);
        servoValueEasyOutputter=sixServoArmController.servoValueOutputter;
        // control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the robot to use these 4 motor names,
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these strings to match your existing Robot
        // Configuration.
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "IntakeLengthMotor");
        output  = hardwareMap.get(DcMotor.class, "OutputLengthMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        output.setDirection(DcMotorSimple.Direction.FORWARD);
//        armPuller = hardwareMap.get(DcMotor.class,"armPuller");
//        armPuller.setDirection(DcMotorSimple.Direction.FORWARD);
//        armPuller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servos3 = hardwareMap.get(Servo.class, "servoc3");
        servos4 = hardwareMap.get(Servo.class, "servoc4");
        servos5 = hardwareMap.get(Servo.class, "servoc5");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        t = System.currentTimeMillis();// 获取当前时间

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward.0
        sixServoArmController.initArm();

    }

    public void fps_and_tele() {


        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps
        telemetry.addData("move_x_l/旋转--逆-顺+", move_x_l);
        telemetry.addData("move_y_l+move_y_r/前-后+", move_y_l + move_y_r);
        telemetry.addData("move_x_r/左-右+", move_x_r);
        telemetry.addData("degree", degree);
        telemetry.addData("clipRadian",clipRadian);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("intake",intake.getCurrentPosition());
        telemetry.addData("output",output.getCurrentPosition());

        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }



    public void runOpMode() {
        inithardware();
        chassisController.initChassis(hardwareMap,gamepad1,gamepad2,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            move_x_l = gamepad1.left_stick_x;
            move_y_l = gamepad1.left_stick_y;
            move_x_r = gamepad1.right_stick_x;
            move_y_r = gamepad1.right_stick_y;
            chassisController.chassisController(move_x_r, -move_y_l,-move_x_l);

            // moveselect(move_x_l,-move_x_r,move_y_l+move_y_r);
            // moveselect(move_x_l,-move_y_l-move_y_r,-move_x_r);
            fps_and_tele();
            motorControl();
            servoControl();
            outputAndInstallerControl();


            /*
            if(gamepad1.x){
                armPuller.setPower(0.1);
            }else if(gamepad1.y){
                armPuller.setPower(-0.1);
            }else{
                armPuller.setPower(0);
            }*/
        }
    }

    //boolean bhasbeenpressed = false, cliplock = false;
    boolean requiresForce=false;
    double installRequiresForcePower=0.3;
    public void motorControl(){
        if(gamepad2.left_trigger > 0.1){
            //intakeLengthController.SingleMotorControl(gamepad2.left_trigger);
            intakeLengthController.SingleMotorControl(gamepad2.left_trigger);//放出滑轨
        }
        else if(gamepad2.right_trigger > 0.1){
            intakeLengthController.SingleMotorControl(-gamepad2.right_trigger);//收回滑轨
        }else if(requiresForce){
            intakeLengthController.SingleMotorControl(-installRequiresForcePower);
        }else{
            intakeLengthController.SingleMotorControl(0);
        }



    }
    double x=0 ,y=150;
    double clipRadian = 0;
    boolean leftstickbuttonhasbeenpressed= false;
    boolean locked=false;
    public static double kx=4;
    public static double ky=4;
    boolean useCombineServoControl = true;
    ServoValueEasyOutputter.ClipPosition CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
    boolean pad2_rbispressed = false;
    boolean ifGivenCommand = false;
    boolean pad2_lstickispressed = false;
    boolean pad2_rstickispressed = false;
    int servo_select = 1;
    double[] servo_position = {0.5,0.5,0.5,0.5,0.5};//伺服位置数组
    boolean p2xIsPressed = false;
    public void servoControl() {
        if(gamepad2.x){
            if(p2xIsPressed){
                p2xIsPressed = true;
                useCombineServoControl =!useCombineServoControl;
            }
        }else{
            p2xIsPressed = false;
        }


        if(useCombineServoControl) {




            //自动操作

            x += gamepad2.left_stick_x * kx;
            y -= gamepad2.left_stick_y * ky;
            if (!gamepad2.dpad_left && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down) {
                if (!gamepad2.left_bumper)
                    sixServoArmController.setTargetPosition(x, y, Math.PI, clipRadian).update();
                else {
                    ArmAction Command = CVModule.CalculateAverage(CVModule);
                    if (Command.suggestion != -2 && !HasSetArmPos) {
                        x = Command.GoToX - 94;
                        y = -Command.GoToY + 44;

                        clipRadian = Command.ClipAngle;
                        HasSetArmPos = true;
                    }
                }
            }
            if (gamepad2.dpad_left) {
                sixServoArmController.scanTheSample();
            }
            if (gamepad2.dpad_up) {
                sixServoArmController.testIfTheSampleIsEaten();//todo 实现该功能
            }
            if (gamepad2.dpad_right) {
                sixServoArmController.dropTheSample();
            }
            if (gamepad2.dpad_down) {
                sixServoArmController.giveTheSample();
                HasSetArmPos = false;
            }else{
                for(int i=0;i<=1;i++){
                    sixServoArmController.giveTheSampleCheckPointInited[i]=false;
                    sixServoArmController.giveTheSampleCheckPointPassed[i]=false;
                }
            }
            ServoValueEasyOutputter.ClipPosition pos;
            boolean changed=false;
            if (gamepad2.left_stick_button) {
                if (!leftstickbuttonhasbeenpressed) {
                    locked = !locked;
                    changed=true;
                    leftstickbuttonhasbeenpressed = true;
                }
            } else {
                leftstickbuttonhasbeenpressed = false;
            }
            if (gamepad2.right_stick_button) {
                changed=true;
            }
            if(changed) {
                if (locked) {
                    pos = ServoValueEasyOutputter.ClipPosition.LOCKED;
                } else {
                    pos = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
                }
                if(gamepad2.right_stick_button)pos = ServoValueEasyOutputter.ClipPosition.HALF_LOCKED;
                servoValueEasyOutputter.setClip(pos);
            }


        }else {






            //手动操作


            if(gamepad2.left_stick_button) {
                if (!pad2_lstickispressed) {
                    servo_select--;
                    pad2_lstickispressed = true;
                    ifGivenCommand = false; // Reset command flag when changing servo selecti
                }
            } else {
                pad2_lstickispressed = false;
            }
            if(gamepad2.right_stick_button) {
                if (!pad2_rstickispressed) {
                    servo_select++;
                    pad2_rstickispressed = true;
                    ifGivenCommand = false;
                }
            } else {
                pad2_rstickispressed = false;
            }
            if (servo_select < 0) {
                servo_select = 4;
            }
            if(servo_select > 4){
                servo_select = 0;
            }

            if(gamepad2.left_stick_x > 0.1 && gamepad2.left_stick_x < 0.6) {
                servo_position[servo_select] += 0.01;
                ifGivenCommand = true;
            }
            else if(gamepad2.left_stick_x < -0.1 && gamepad2.left_stick_x > -0.6) {
                servo_position[servo_select] -= 0.01;
                ifGivenCommand = true;
            }
            if(gamepad2.left_stick_x > 0.6) {
                servo_position[servo_select] += 0.08;
                ifGivenCommand = true;
            }
            else if(gamepad2.left_stick_x <-0.6) {
                servo_position[servo_select] -= 0.08;
                ifGivenCommand = true;
            }

            if (servo_position[servo_select] > 1)
                servo_position[servo_select] = 1;
            if (servo_position[servo_select] < 0)
                servo_position[servo_select] = 0;
            if(ifGivenCommand){
                servoValueEasyOutputter.SingleServoControl(servo_select, servo_position[servo_select]);
            }
            if (gamepad2.right_bumper) {
                if (!pad2_rbispressed) {
                    t = System.currentTimeMillis();
                    pad2_rbispressed = true;
                }
                if (System.currentTimeMillis() - t > 1000) {
                    CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.HALF_LOCKED;
                }
            } else {
                if (pad2_rbispressed) {
                    if (System.currentTimeMillis() - t < 500) {
                        if (CurrentClipPosition == ServoValueEasyOutputter.ClipPosition.LOCKED) {
                            CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
                        } else {
                            CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.LOCKED;
                        }
                    }
                }

                pad2_rbispressed = false;
            }
            servoValueEasyOutputter.setClip(CurrentClipPosition);
        }
    for(int i=0;i<=4;i++){
        servo_position[i]=servoValueEasyOutputter.servoNowDegree[i];
    }
        //intakeLengthController.setIntakeTargetPosition(servoValueEasyOutputter.giveTheSample(servoValueEasyOutputter.InstallerLocationX,servoValueEasyOutputter.InstallerLocationY,servoValueEasyOutputter.InstallerLocationZ)).update();
    }
    InstallerController installerController;
    OutputController outputController;
    boolean outputInited = false;
    public double OutputstartTime = 0;
    public static double OutputTime = 300;
    boolean gp1Xpressed=false;
    public void outputAndInstallerControl() {
        if(gamepad1.x){
            if(!gp1Xpressed){
                outputController.setMode(outputController.outputStates.next());
                gp1Xpressed=true;
            }
        }else{
            gp1Xpressed=false;
        }
        if(gamepad1.left_stick_button){
            outputController.ArmUp();
        }
        if(gamepad1.right_stick_button){
            outputController.ArmDown();
        }
        if(gamepad1.dpad_up){
            outputController.setClip(false);
        }
        if(gamepad1.dpad_down){
            outputController.setClip(true);
        }
        if (gamepad1.b) {
            installerController.BeamSpinner(false);
        }
        if (gamepad1.a) {
            installerController.BeamSpinner(true);
        }


        if (gamepad2.a) {
            installerController.setMode(RobotStates.INSTALL_RUNMODE.EATING);
        }
        if (gamepad2.y) {
            outputInited = false;
            installerController.Install();
        }
        installerController.run();

        if (gamepad2.b) {
            installerController.setMode(RobotStates.INSTALL_RUNMODE.BACKING);
            if (!outputInited) {
                OutputstartTime = System.currentTimeMillis();
                outputInited = false;
            }

            if (System.currentTimeMillis() - OutputstartTime > OutputTime) {
                CurrentClipPosition = ServoValueEasyOutputter.ClipPosition.UNLOCKED;
                outputController.ArmUp();
            }
        }
    }
}