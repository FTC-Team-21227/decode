package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

public class AprilTagMecanumDrive extends MecanumDrive{
    Camera camera;
    //localizer functions reset with apriltags
    public AprilTagMecanumDrive(HardwareMap hardwareMap, Pose2d initialPose, Camera camera){
        super(hardwareMap,initialPose);
        this.camera = camera;
    }
    public Pose2d relocalize(Telemetry telemetry) {
        PoseVelocity2d vel = super.updatePoseEstimate();
        if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
            return localizer.getPose();
        }
        Pose2d pose = camera.update(telemetry);
        localizer.setPose(pose);
        return pose;
    }
}
