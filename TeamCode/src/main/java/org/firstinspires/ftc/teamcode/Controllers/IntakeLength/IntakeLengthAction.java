package org.firstinspires.ftc.teamcode.Controllers.IntakeLength;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeLengthAction{

    HardwareMap hardwareMap;
    IntakeLengthControllerInterface intakeLengthController;
    public IntakeLengthAction(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        intakeLengthController  = new MotorLineIntakeLengthController(hardwareMap);
    }
    double targetPosition;

    public class IntakeRunToPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            intakeLengthController.setIntakeTargetPosition(targetPosition).update();
            return !(intakeLengthController.getIntakeLengthCurrentPosition() == intakeLengthController.getIntakeLengthTargetPosition());//true=continue , false = finish
        }

    }
    public Action intakeRunToPosition(double targetPosition){
        this.targetPosition=targetPosition;
        return new IntakeRunToPosition();
    }

    public class IntakeLengthInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            intakeLengthController.setIntakeTargetPosition(0).update();
            return !(intakeLengthController.getIntakeLengthCurrentPosition() == intakeLengthController.getIntakeLengthTargetPosition());
        }

    }
    public Action intakeLengthInit(){

        return new IntakeLengthInit();
    }
}
