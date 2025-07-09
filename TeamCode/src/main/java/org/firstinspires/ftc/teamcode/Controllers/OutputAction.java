package org.firstinspires.ftc.teamcode.Controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OutputAction {
    HardwareMap hardwareMap;
    public OutputAction(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    class OutputArmRunToPosition implements Action {
        double targetHeight;
        OutputController outputController = OutputController.getInstance(hardwareMap);
        public OutputArmRunToPosition(double targetHeight){
            this.targetHeight = targetHeight;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return outputController.setTargetOutputHeight(targetHeight).update();
        }
    }
    public Action OutputArmRunToPosition(double targetHeight){
        return new OutputArmRunToPosition(targetHeight);
    }
}
