//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.hardware.*;
//import com.acmerobotics.roadrunner.*;
//public class UselessSamples {
//
//        public class Lift {
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class LiftUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos < 3000.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//
//
//        }
//        public Action liftUp() {
//            return new LiftUp();
//        }
//
//        public class LiftDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(-0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > 100.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action liftDown(){
//            return new LiftDown();
//        }
//    }
//
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            //claw = hardwareMap.get(Servo.class, "claw");
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(0.55);
//                return false;
//            }
//        }
//        public Action closeClaw() {
//            return new CloseClaw();
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(1.0);
//                return false;
//            }
//        }
//        public Action openClaw() {
//            return new OpenClaw();
//        }
//    }
//
//
//
//
//
//
//    //                    new SequentialAction(
////                            trajectoryActionChosentest,
////                              armController.outPut(),
////                            //claw.openClaw(),
////                            //lift.liftDown(),
////                            trajectoryActionCloseOutTest
////                    ),
//
//
//    Action trajectoryActionChosentest;
//            if (startPosition == 1) {
//        trajectoryActionChosentest = tab1.build();
//    } else if (startPosition == 2) {
//        trajectoryActionChosentest = tab2.build();
//    } else if (startPosition == 3) {
//        trajectoryActionChosentest = tab3.build();
//    } else {
//        trajectoryActionChosentest = tab4.build();
//    }
//
//
//    TrajectoryActionBuilder Tap1 = drive.actionBuilder(new Pose2d(-25.96, -68.39, Math.toRadians(90.00)))
//            .splineTo(new Vector2d(-35.44, -25.05), Math.toRadians(178.64))
//            .splineTo(new Vector2d(-60.49, -59.13), Math.toRadians(225.00));
//    Action CloseOutTap1 = Tap1.endTrajectory().fresh()
//            .build();
//    Action ActionTap1 = Tap1.build();
//
//    TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//            .lineToYSplineHeading(33, Math.toRadians(0))
//            .waitSeconds(2)
//            .setTangent(Math.toRadians(90))
//            .lineToY(48)
//            .setTangent(Math.toRadians(0))
//            .lineToX(32)
//            .strafeTo(new Vector2d(44.5, 30))
//            .turn(Math.toRadians(180))
//            .lineToX(47.5)
//            .waitSeconds(3);
//    TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//            .lineToY(37)
//            .setTangent(Math.toRadians(0))
//            .lineToX(18)
//            .waitSeconds(3)
//            .setTangent(Math.toRadians(0))
//            .lineToXSplineHeading(46, Math.toRadians(180))
//            .waitSeconds(3);
//    TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//            .lineToYSplineHeading(33, Math.toRadians(180))
//            .waitSeconds(2)
//            .strafeTo(new Vector2d(46, 30))
//            .waitSeconds(3);
//    TrajectoryActionBuilder tab4 =
//            drive.actionBuilder(initialPose)
//                    .strafeTo(new Vector2d(-60, -70.86));
//    .strafeToLinearHeading(new Vector2d(100,100), new Rotation2d(0,1));
//    .splineToConstantHeading(new Vector2d(10,10), new Rotation2d(0,0));
//    .lineToY(10);
//    .strafeTo(new Vector2d(10,10));
//
//    Action trajectoryActionCloseOut = tab4.endTrajectory().fresh()
//            //.strafeTo(new Vector2d(48, 12))
//            .build();
//}
