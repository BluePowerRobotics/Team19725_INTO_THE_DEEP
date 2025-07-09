package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeLengthAction{

    HardwareMap hardwareMap;
    public IntakeLengthAction(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        IntakeLengthController.getInstance(hardwareMap);
    }
    double targetPosition;

    public class IntakeRunToPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            IntakeLengthController.getInstance().setIntakeTargetPosition(targetPosition).update();
            return !(IntakeLengthController.getInstance().getIntakeLengthNowRadian() == IntakeLengthController.getInstance().getIntakeLengthTargetRadian());//true=continue , false = finish
        }

    }
    public Action intakeRunToPosition(double targetPosition){
        this.targetPosition=targetPosition;
        return new IntakeRunToPosition();
    }

    public class IntakeLengthInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            IntakeLengthController.getInstance().setIntakeTargetPosition(0).update();
            return !(IntakeLengthController.getInstance().getIntakeLengthNowRadian() == IntakeLengthController.getInstance().getIntakeLengthTargetRadian());
        }

    }
    public Action intakeLengthInit(){

        return new IntakeLengthInit();
    }
}
