package org.firstinspires.ftc.teamcode.Controllers.IntakeLength;

public interface IntakeLengthControllerInterface {
    IntakeLengthControllerInterface setIntakeTargetPosition(double inTakeLength);
    IntakeLengthControllerInterface update();
    double getIntakeLengthCurrentPosition();
    double getIntakeLengthTargetPosition();
}
