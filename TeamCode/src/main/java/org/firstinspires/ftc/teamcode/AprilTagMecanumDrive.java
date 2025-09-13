package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagMecanumDrive extends MecanumDrive{
    //localizer functions reset    with apriltags
    public AprilTagMecanumDrive(HardwareMap hardwareMap, Pose2d initialPose){
        super(hardwareMap,initialPose);
    }
}
