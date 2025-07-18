package org.firstinspires.ftc.teamcode.Controllers.OutPut;

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
    class OutputEatIntake implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).eatIntake();
        }
    }
    public Action OutputEatIntake(){
        return new OutputEatIntake();
    }
    class OutputVomitInstaller implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).vomitInstaller();
        }
    }
    public Action OutputVomitInstaller(){
        return new OutputVomitInstaller();
    }
    class OutputEatInstaller implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).eatInstaller();
        }
    }
    public Action OutputEatInstaller(){
        return new OutputEatInstaller();
    }
    class OutputThrowAwaySample implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return OutputController.getInstance(hardwareMap).throwAwaySample();
        }
    }
    public Action OutputThrowAwaySample(){
        return new OutputThrowAwaySample();
    }
}
