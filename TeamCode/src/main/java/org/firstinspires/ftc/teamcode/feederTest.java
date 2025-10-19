package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Feeder Test")
// Program used to test hood positions
public class feederTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Feeder feeder;

    double feederPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                feederPos = 0;
            }
            if (gamepad1.yWasPressed()) {
                feederPos = 1;
            }
            if (gamepad1.dpadUpWasPressed()) {
                feederPos += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()) {
                feederPos -= 0.01;
            }
            feeder.FEEDER.setPosition(feederPos);
            // Telemetry lines
            telemetry.addData("feeder target pos", feederPos);
            telemetry.addData("feeder pos", feeder.FEEDER.getPosition());
            telemetry.update();
        }
    }

    private void initialization() {
        feeder = new Feeder(hardwareMap);
    }
}
