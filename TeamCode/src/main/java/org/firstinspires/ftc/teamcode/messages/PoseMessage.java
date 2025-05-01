package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double tmp_x;
    public double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.tmp_x = pose.position.x *100;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }
}

