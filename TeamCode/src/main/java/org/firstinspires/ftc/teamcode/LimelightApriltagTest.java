package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LimelightApriltagTest", group = "Concept")
public class LimelightApriltagTest extends OpMode{

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // limelight 0 pipeline
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); // MIGHT CHANGE
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start()
    {
        limelight.start();
    }

    @Override
    public void loop()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        double robotYaw = orientation.getYaw();
        limelight.updateRobotOrientation(robotYaw);
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose_mt2 = llResult.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
//        if (llResult!=null && llResult.isValid())
//        {
//            Pose3D botpose = llResult.getBotpose();
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//            telemetry.addData("BotPose", botpose.toString());
//            telemetry.update();
//        }

    }
}
