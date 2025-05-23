/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/*import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;*/
import com.qualcomm.robotcore.hardware.HardwareMap;

//单独移动测试
public class ChassisController {
    GoBildaPinpointDriver odo;
    double leftFrontPower;
    double leftBackPower;
    double rightBackPower;
    double rightFrontPower;
    double speed;
    double alpha;
    double omiga;
    double angle;
    double getthita;
    double setthita;
    double px;
    double py;
    double lock_thita;


    public enum MODE{RUN_TO_LOCATION,RUN_WITHOUT_LOCATOR,STOP_AND_RESET_LOCATOR}
    private MODE RUNMODE = ChassisController.MODE.RUN_WITHOUT_LOCATOR;


    private boolean UseAutoMove = false;


    private int targetLocationX=0;
    private int targetLocationY=0;
    private int targetLocationR=0;
    private double distanceToTargetLocation=0;

    public void setMode(MODE CHASSIS_MODE){
        RUNMODE = CHASSIS_MODE;


        if(RUNMODE==MODE.STOP_AND_RESET_LOCATOR){
            UseAutoMove = true;

            initLocator();


        } else if (RUNMODE==MODE.RUN_TO_LOCATION) {
            UseAutoMove = true;
            freshThita();
            runToLocation(targetLocationX,targetLocationY,targetLocationR,0,maxSpeed);
        } else if (RUNMODE==MODE.RUN_WITHOUT_LOCATOR){
            UseAutoMove = false;
        } else{
            UseAutoMove = false;
            RUNMODE = MODE.RUN_WITHOUT_LOCATOR;
        }
    }
    public static final int NO_HEAD_MODE = 1;
    public static final int ORDINARY_MODE = 0;
    public static final int EXCHANGE_MODE = 2;
    public void setRunMode(int RunMode){
        if(RunMode == NO_HEAD_MODE){
            USE_NO_HEAD_MODE = true;
        }else if(RunMode == ORDINARY_MODE){
            USE_NO_HEAD_MODE = false;
        }else if(RunMode == EXCHANGE_MODE){
            USE_NO_HEAD_MODE = !USE_NO_HEAD_MODE;
        }
    }

    public void setTargetLocation(int targetX,int targetY,int targetR){
        targetLocationX=targetX;
        targetLocationY=targetY;
        targetLocationR=targetR;
        distanceToTargetLocation = Math.sqrt(targetLocationX^2+targetLocationY^2);
    }
    double maxSpeed;
    public void setSpeed(double Speed){
        maxSpeed= Speed;
    }



    public boolean climb = false;

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection
            .values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection
            .values();
    HardwareMap hardwareMap;
    Gamepad gamepad1,gamepad2;
    DcMotor a124;
    IMU imu;
    YawPitchRollAngles orientation;
    static DcMotor leftFront, leftBack, rightBack, rightFront, armMotor;
    int logoFacingDirectionPosition = 0;
    int usbFacingDirectionPosition = 2;

    public boolean USE_NO_HEAD_MODE = false;
    public boolean yhasbeenpressed = false;
    public boolean thitalock = false;
    public boolean xhasbeenpressed = false;
    public double thita = 0;
    public double angletime = 0;

    public void initChassis(HardwareMap hardwareMaprc, Gamepad gamepad1rc, Gamepad gamepad2rc) {

        hardwareMap = hardwareMaprc;
        gamepad1 = gamepad1rc;
        gamepad2 = gamepad2rc;
        imu = hardwareMap.get(IMU.class, "eimu");
        orientation = imu.getRobotYawPitchRollAngles();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // Configure the robot to use these 4 motor names,
        leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // or change these strings to match your existing Robot
                                                               // Configuration.
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        freshThita();
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

    public void freshThita() {
        updateOrientation();
        thita = orientation.getYaw(AngleUnit.DEGREES);
    }

    public void move(double r, double y, double x) {
        // r,y,x
        /*
         * if(r<=0.1 && r>=-0.1) r=0;
         * if(y<=0.1 && y>=-0.1) y=0;
         * if(x<=0.1 && x>=-0.1) x=0;
         */
        double gen2 = 0.7071068;// 二分之根号二
        /*
         * OP20240325_test1G.leftFront.setPower(-gen2 * (r + y)+x);
         * OP20240325_test1G.leftBack.setPower(-gen2 * (r + y)-x);
         * OP20240325_test1G.rightBack.setPower(-gen2 * (r - y)+x);
         * OP20240325_test1G.rightFront.setPower(-gen2 * (r - y)-x);
         */

        leftFrontPower = gen2 * (+x - y) - r;
        leftBackPower = gen2 * (-x - y) - r;
        rightBackPower = gen2 * (+x - y) + r;
        rightFrontPower = gen2 * (-x - y) + r;

        if (leftFrontPower < 0.05 && leftFrontPower > -0.05)
            leftFrontPower = 0;
        if (leftBackPower < 0.05 && leftBackPower > -0.05)
            leftBackPower = 0;
        if (rightBackPower < 0.05 && rightBackPower > -0.05)
            rightBackPower = 0;
        if (rightFrontPower < 0.05 && rightFrontPower > -0.05)
            rightFrontPower = 0;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

    }
    public double multiplyX = 1;
    public double getNowX(){
        return -odo.getEncoderY()*multiplyX;
    }
    public double multiplyY = 1;
    public double getNowY(){
        return odo.getEncoderX()*multiplyY;
    }
    public void initLocator(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        
        odo.resetPosAndIMU();
    }

    public void runToLocation(double targetX, double targetY, double targetR,double pathError,double maxSpeed) {
        // x横向，y竖向，r旋转（-1~1），speed速
        // t目标，n目前
        // d距离
        // 半场1.78m，以m为单位
        // Math.sqrt()
        double nowX = getNowX();
        double nowY = getNowY();
        double errorX = targetX - nowX;
        double errorY = targetY - nowY;
        double speed = Math.sqrt(errorX*errorX+errorY*errorY);
        double outSpeed = Math.min(speed,maxSpeed);
        double pathDegree,moveX,moveY;
        if (errorX != 0 || errorY != 0) {
            pathDegree = Math.toDegrees(Math.atan(Math.abs(errorY / errorX)));
            if (errorY <= 0 && errorX > 0)
                pathDegree = 360 - pathDegree;
            if (errorY <= 0 && errorX <= 0)
                pathDegree = 180 + pathDegree;
            if (errorY > 0 && errorX <= 0)
                pathDegree = 180 - pathDegree;

            moveX = outSpeed * Math.cos(Math.toRadians(pathDegree+pathError*speed/distanceToTargetLocation));
            // if (getthita<=0) px = -px;
            moveY = outSpeed * Math.sin(Math.toRadians(pathDegree+pathError*speed/distanceToTargetLocation));
        } else {
            moveX = 0;
            moveY = 0;
        }
        lock_thita = targetR;
        // double x=speed*dx/(Math.sqrt(dx*dx+dy*dy));
        // double y=speed*dy/(Math.sqrt(dx*dx+dy*dy));
        noheadmove(thitalock(), moveY, moveX, 2);

    }

    public double setspin(double r, double x) {
        if (x > 0)
            r = -r;// 手操后退时转向改变，即将r为负与正改为绕左侧旋转&绕右侧旋转
        return r;
    }

    public void noheadmove(double nr, double ny, double nx, double ns) {
        noheadmove(nr, ny, nx, ns, false);
    }

    public void noheadmove(double nr, double ny, double nx, double ns, boolean spinchange) {

        speed = ns * Math.sqrt(nx * nx + ny * ny);
        if (nx != 0 || ny != 0) {

            alpha = Math.toDegrees(Math.atan(Math.abs(ny / nx)));
            if (ny <= 0 && nx > 0)
                alpha = 360 - alpha;
            if (ny <= 0 && nx <= 0)
                alpha = 180 + alpha;
            if (ny > 0 && nx <= 0)
                alpha = 180 - alpha;
            omiga = -getthita + setthita;
            if (omiga < 0)
                omiga = 360 + omiga;
            angle = alpha - omiga;
            if (angle < 0)
                angle = 360 + angle;
            px = speed * Math.cos(Math.toRadians(angle));
            // if (getthita<=0) px = -px;
            py = speed * Math.sin(Math.toRadians(angle));
        } else {
            py = 0;
            px = 0;
        }

        if (spinchange)
            move(setspin(nr, py), py, px);// px->py
        else
            move(nr, py, px);
    }

    public double thitalock() {
        double locked_thita = (getthita - lock_thita);
        while (locked_thita < -180) {
            locked_thita += 360;
        }
        while (locked_thita > 180) {
            locked_thita -= 360;
        }
        if (locked_thita < 0) {
            locked_thita = -((-locked_thita) % 360);
        } else {
            locked_thita = locked_thita % 360;
        }
        locked_thita = locked_thita / 45;
        // if (locked_thita < -2) locked_thita +=4;
        // if (locked_thita > 2) locked_thita -=4;
        return locked_thita;
    }
    public boolean USE_SLOW_MODE = false;

    public void chassisController(double r, double y, double x){
        double receiveSpeed = 1;
        if (gamepad1.x) {
            if (!xhasbeenpressed) {
                USE_SLOW_MODE = !USE_SLOW_MODE;
            }
            xhasbeenpressed = true;
        } else {
            xhasbeenpressed = false;
        }
        if(USE_SLOW_MODE) receiveSpeed = 0.6;
        chassisController(r,y,x,receiveSpeed);
    }
    public void chassisController(double r, double y, double x, double speed) {
        if(!UseAutoMove) {
            freshThita();
            if (gamepad1.y) {
                if (!yhasbeenpressed) {
                    if (!USE_NO_HEAD_MODE)
                        USE_NO_HEAD_MODE = true;
                    else if (USE_NO_HEAD_MODE)
                        USE_NO_HEAD_MODE = false;
                    yhasbeenpressed = true;
                } else {
                    yhasbeenpressed = true;
                }
                setthita = thita;

            } else {
                yhasbeenpressed = false;
            }

            if (gamepad1.x) {
                if (!xhasbeenpressed) {
                    if (!thitalock)
                        thitalock = true;
                    else if (thitalock)
                        thitalock = false;
                    xhasbeenpressed = true;
                } else {
                    xhasbeenpressed = true;
                }
                lock_thita = thita;
            } else {
                xhasbeenpressed = false;
            }

            getthita = thita;

            if (thitalock) {
                r = thitalock();
            } else {
                if (System.currentTimeMillis() - angletime >= 10) {
                    if (y < 0) {
                        r = -r;
                    }
                    lock_thita -= 2 * r;
                    angletime = System.currentTimeMillis();
                }
                r = thitalock();
            }
            if (gamepad1.left_bumper)
                lock_thita = 45;
            else if (gamepad1.right_bumper)
                lock_thita = 90;

            if (!USE_NO_HEAD_MODE || climb) {
                move(r, y, x);
            } else {
                noheadmove(r, y, x, speed);
            }
        }
    }
}
