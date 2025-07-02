package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.ChassisController;
import org.firstinspires.ftc.teamcode.Controllers.ClimbController;
import org.firstinspires.ftc.teamcode.Controllers.RobotStates;

@TeleOp

public class OP03252 extends LinearOpMode {
    //MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-24,-48,Math.toRadians(180)));
    org.firstinspires.ftc.teamcode.Controllers.ChassisController ChassisController = new ChassisController();// 构建class实例
    org.firstinspires.ftc.teamcode.Controllers.ArmController ArmController = new ArmController(hardwareMap, telemetry);
    org.firstinspires.ftc.teamcode.Controllers.ClimbController ClimbController = new ClimbController();
    //static DcMotor leftFront, leftBack, rightBack, rightFront, armMotor;
    //Servo servoe3, servoe4, servoe5;
    MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-24,-48,Math.toRadians(180)));
    ChassisController ChassisController = new ChassisController();// 构建class实例
    ArmController ArmController = new ArmController();
    ClimbController ClimbController = new ClimbController();
    static DcMotor leftFront, leftBack, rightBack, rightFront, armMotor;
    Servo servoe3, servoe4, servoe5;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection
            .values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection
            .values();

    IMU imu;
    int logoFacingDirectionPosition = 0;
    int usbFacingDirectionPosition = 2;
    boolean orientationIsValid = true;
    YawPitchRollAngles orientation;

    public double angletime = 0;

    public double t = 0;// 当前时间
    public double move_x_l;
    public double move_y_l;
    public double move_x_r;
    public double move_y_r;
    public double move_x2;
    public double degree = 0;
    public double thita = 0;

    public boolean noheadmode = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;
    public boolean lthasbeenpressed = false, rthasbeenpressed = false;
    /*
     * servoe5:0.86~1
     * servoe4:0;0.515;0.95
     * servoe3:0.7~1
     */

    private void inithardware() {
        // control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        // expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        /*
         * leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the
         * robot to use these 4 motor names,
         * leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these
         * strings to match your existing Robot Configuration.
         * rightBack = hardwareMap.get(DcMotor.class, "rightBack");
         * rightFront = hardwareMap.get(DcMotor.class, "rightFront");
         * servoe3 = hardwareMap.get(Servo.class, "servoe3");
         * servoe4 = hardwareMap.get(Servo.class, "servoe4");
         * servoe5 = hardwareMap.get(Servo.class, "servoe5");
         * leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         * 
         * rightFront.setDirection(DcMotor.Direction.REVERSE);
         * leftFront.setDirection(DcMotor.Direction.FORWARD);
         * rightBack.setDirection(DcMotor.Direction.REVERSE);
         * leftBack.setDirection(DcMotor.Direction.FORWARD);
         * 
         */
        t = System.currentTimeMillis();// 获取当前时间

        // imu = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "eimu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        orientation = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        updateOrientation();

    }

    public void fps_and_tele() {

        telemetry.addData("fps", 1000 / (System.currentTimeMillis() - t));// fps


        telemetry.addData("move_x_l/旋转--逆-顺+", move_x_l);
        telemetry.addData("move_y_l+move_y_r/前-后+", move_y_l + move_y_r);
        telemetry.addData("move_x_r/左-右+", move_x_r);
        telemetry.addData("degree", degree);

        telemetry.addData("px", ChassisController.px);
        telemetry.addData("py", ChassisController.py);
        telemetry.addData("omiga", ChassisController.omiga);
        telemetry.addData("alpha", ChassisController.alpha);
        telemetry.addData("angle", ChassisController.angle);

        telemetry.addData("servo_position", ArmController.servo_position);

        telemetry.addData("length", ArmController.motorNowLength / ArmController.motorLength);

        telemetry.update();
        t = System.currentTimeMillis();// 获取当前时间
    }

    void updateOrientation() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        /*
         * try {
         * RevHubOrientationOnRobot orientationOnRobot = new
         * RevHubOrientationOnRobot(logo, usb);
         * imu.initialize(new IMU.Parameters(orientationOnRobot));
         * orientationIsValid = true;
         * } catch (IllegalArgumentException e) {
         * orientationIsValid = false;
         * }
         */
        orientation = imu.getRobotYawPitchRollAngles();
    }
    public boolean armInitFinished = false;
    public void runOpMode() {
        inithardware();
        RobotStates robotStates = RobotStates.getInstance();
        RobotStates.getInstance().setAUTO(false);
        RobotStates.getInstance().setCLIMBING(false);
        RobotStates.getInstance().setMODE(RobotStates.RUNMODE.HIGH_CHAMBER);
        ChassisController.initChassis(hardwareMap, gamepad1,gamepad2,telemetry);

        ClimbController.initClimb(hardwareMap,gamepad2,telemetry);
        while(!armInitFinished) armInitFinished = ArmController.initArm(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            /*
             * updateOrientation();
             * thita = orientation.getYaw(AngleUnit.DEGREES);
             */

            move_x_l = gamepad1.left_stick_x + gamepad2.left_stick_x;
            move_y_l = gamepad1.left_stick_y + gamepad2.left_stick_y;
            move_x_r = gamepad1.right_stick_x + gamepad2.right_stick_x;
            move_y_r = gamepad1.right_stick_y + gamepad2.right_stick_y;
            //r,y,x,speed
            ChassisController.chassisController(move_x_l, -move_x_r, move_y_l + move_y_r);
            ArmController.armController();
            ClimbController.climb();
            fps_and_tele();


        }
    }

    /*
     * public void moveselect(double r, double y, double x){
     * if (gamepad1.y){
     * if (!yhasbeenpressed){
     * if (!noheadmode) noheadmode = true;
     * else if (noheadmode) noheadmode = false;
     * yhasbeenpressed = true;
     * }else{
     * yhasbeenpressed = true;
     * }
     * rbmove.setthita = thita;
     * 
     * }else{
     * yhasbeenpressed = false;
     * }
     * 
     * if (gamepad1.x){
     * if (!xhasbeenpressed){
     * if (!thitalock) thitalock = true;
     * else if (thitalock) thitalock = false;
     * xhasbeenpressed = true;
     * }else{
     * xhasbeenpressed = true;
     * }
     * rbmove.lock_thita = thita;
     * }else{
     * xhasbeenpressed = false;
     * }
     * 
     * rbmove.getthita = thita;
     * 
     * if(thitalock){
     * r=rbmove.thitalock();
     * }else{
     * if(System.currentTimeMillis()-angletime>=10) {
     * if(y<0){
     * r = -r;
     * }
     * rbmove.lock_thita -= 2*r;
     * angletime = System.currentTimeMillis();
     * }
     * r=rbmove.thitalock();
     * }
     * if(gamepad1.left_bumper) rbmove.lock_thita=45;
     * else if(gamepad1.right_bumper) rbmove.lock_thita =0;
     * 
     * 
     * 
     * if (!noheadmode){
     * rbmove.move(r,y,x);
     * }else{
     * rbmove.noheadmove(r,y,x,1);
     * }
     * }
     */

    // ArmController
    /*
     * boolean bhasbeenpressed = false, cliplock = false;
     * boolean armup = false;
     * boolean ahasbeenpressed = false;
     * double servo_position = 0.9;
     * 
     * double motorTime = 0;
     * double motorLength=100000;
     * double motorNowLength = 0;
     * double motorPower = 0;
     * 
     * double clipLockPos=0.7;
     * double clipUnlockPos=1;
     * double clipUpPos=0.3;
     * double clipDownPos=0.715;
     * double armUpPos=0;
     * double armPosMax=1;
     * double armPosMin=0.7;
     * double[] armMotorPosition;
     * 
     * void initArm(){
     * armMotor = hardwareMap.get(DcMotor.class, "armMotor");
     * armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     * 
     * cliplock = false;
     * armup = false;
     * servo_position = 0.9;//default position when arm is down.
     * 
     * motorTime = 0;
     * motorLength=600;
     * motorNowLength = 0;
     * motorPower = 0;
     * 
     * clipLockPos=0.69;
     * clipUnlockPos=0.92;
     * clipUpPos=0.2;
     * clipDownPos=0.715;
     * armUpPos=0.1;
     * armPosMax=1;
     * armPosMin=0.7;
     * 
     * //armUp
     * servoe3.setPosition(armUpPos);//一级舵机竖直
     * servoe4.setPosition(clipUpPos);//二级舵机向后
     * 
     * armMotor.setPower(1);
     * 
     * while(System.currentTimeMillis()-t<=2000){
     * armMotor.setTargetPosition(560*2);
     * armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * telemetry.addData("Testing", armMotor.getCurrentPosition());
     * telemetry.update();
     * }
     * 
     * armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     * telemetry.addData("StartToBack", armMotor.getCurrentPosition());
     * telemetry.update();
     * while(armMotor.getCurrentPosition()+motorLength>5 ||
     * armMotor.getCurrentPosition()+motorLength<-5){
     * armMotor.setTargetPosition(-(int)motorLength);
     * armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * telemetry.addData("Backing", armMotor.getCurrentPosition());
     * telemetry.update();
     * }
     * armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     * armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * armLenthControl();
     * armCalculator();
     * armDown();
     * //armMotor.setMode(DcMotor.RunMode.Run_WITHOUT_ENCODER);
     * telemetry.addData("Finished", armMotor.getCurrentPosition());
     * telemetry.update();
     * }
     * 
     * 
     * void gampadArmReceiver(){
     * //clip
     * if (gamepad1.b){
     * if (!bhasbeenpressed){
     * if (!cliplock){
     * cliplock = true;
     * clipLockTime = System.currentTimeMillis()+500;
     * }
     * else if (cliplock) cliplock = false;
     * bhasbeenpressed = true;
     * }else{
     * bhasbeenpressed = true;
     * }
     * }else{
     * bhasbeenpressed = false;
     * }
     * //一级控制
     * 
     * //if (gamepad1.left_bumper){
     * // servo_position +=0.005;
     * //}
     * //if (gamepad1.right_bumper){
     * // servo_position -=0.005;
     * //}
     * //机械臂模式选择
     * if (gamepad1.a){
     * if (!ahasbeenpressed){
     * if(armup) {
     * armup=false;
     * armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * }else {
     * armup = true;
     * armUpTime = t +1500;
     * armMotor.setTargetPosition(0);
     * armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * armMotor.setPower(1);
     * }
     * ahasbeenpressed = true;
     * }
     * }else{
     * ahasbeenpressed = false;
     * }
     * }
     * 
     * double armUpTime=0;
     * void armUp(){
     * 
     * servoe3.setPosition(armUpPos);//一级舵机竖直
     * servoe4.setPosition(clipUpPos);//二级舵机向后
     * if(armUpTime>=System.currentTimeMillis()){
     * armMotor.setTargetPosition(0);
     * armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * //armMotor.setPower(1);
     * }else{
     * armMotor.setTargetPosition(540);
     * armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * //armMotor.setPower(1);
     * }
     * }
     * void armDown(){
     * armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     * servoe3.setPosition(servo_position);//一级舵机地面
     * servoe4.setPosition(clipDownPos);//二级舵机向下
     * }
     * 
     * 
     * void angleRange(){
     * //arm角度范围限定
     * if(servo_position>armPosMax) servo_position = armPosMax;
     * if(servo_position<armPosMin) servo_position = armPosMin;
     * //arm长度范围限定
     * //motorNowLength = Math.max(0,Math.min(motorLength,motorNowLength));
     * //clip角度范围限定
     * clipDownPos = Math.max(0.29, Math.min(0.715, clipDownPos));
     * 
     * }
     * 
     * double clipLockTime=0;
     * void clipControl(){
     * if(cliplock && clipLockTime<=System.currentTimeMillis())
     * servoe5.setPosition(clipLockPos);
     * else servoe5.setPosition(clipUnlockPos);
     * }
     * 
     * 
     * public void armLenthControl(){
     * if(!armup){
     * if(gamepad1.left_trigger >=0.5 && gamepad1.right_trigger <0.5 &&
     * motorNowLength>0) motorPower = -0.5;
     * else if(gamepad1.left_trigger < 0.5 && gamepad1.right_trigger >=0.5 &&
     * motorNowLength<motorLength) motorPower = 0.5;
     * else motorPower = 0;
     * armMotor.setPower(motorPower);
     * }
     * //if (motorPower!=0){
     * // motorNowLength += motorPower*(System.currentTimeMillis()-motorTime);
     * //}
     * motorNowLength = armMotor.getCurrentPosition();
     * 
     * telemetry.addData("armPos",armMotor.getCurrentPosition());
     * motorTime = System.currentTimeMillis();
     * }
     * 
     * void armCalculator(){
     * double L = 30.5 + (motorNowLength / motorLength) * 32.0;
     * double argument = -18.0 / L;
     * servo_position = Math.toDegrees(Math.acos(argument)) / 135.0 ;
     * if(cliplock && clipLockTime+500>=System.currentTimeMillis())
     * servo_position+=0.12;
     * clipDownPos = (3*0.29+2-1.5*(servo_position-0.2))/3;
     * }
     * 
     * public void armController(){
     * gampadArmReceiver();
     * armLenthControl();
     * armCalculator();
     * if (armup){
     * armUp();
     * }else{
     * armDown();
     * }
     * clipControl();
     * }
     */
}
