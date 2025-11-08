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
        telemetry.addLine("Started!");
        telemetry.update();
        sleep(500);
        while (opModeIsActive()) {
            telemetry.addData("A pressed?", gamepad1.a);
            telemetry.addData("Y pressed?", gamepad1.y);
            telemetry.update();

            if (gamepad1.a) {
//                feederPos = 0;
//                feeder.down(feeder.FR_FEEDER);
                feeder.upFR();
            }
            if (gamepad1.y) {
//                feederPos = 1;
//                feeder.up(feeder.FR_FEEDER);
                feeder.downFR();
            }
            if (gamepad1.x) {
//                feederPos = 0;
//                feeder.down(feeder.FR_FEEDER);
                feeder.upBL();
            }
            if (gamepad1.b) {
//                feederPos = 1;
//                feeder.up(feeder.FR_FEEDER);
                feeder.downBL();
            }
            if (gamepad1.dpadUpWasPressed()) {
                feeder.BL_FEEDER.setPosition(feeder.BL_FEEDER.getPosition() + 0.01);
            }
            if (gamepad1.dpadDownWasPressed()) {
                feeder.BL_FEEDER.setPosition(feeder.BL_FEEDER.getPosition() - 0.01);
            }
            if (gamepad1.dpadRightWasPressed()) {
                feeder.FR_FEEDER.setPosition(feeder.FR_FEEDER.getPosition() + 0.01);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                feeder.FR_FEEDER.setPosition(feeder.FR_FEEDER.getPosition() - 0.01);
            }
//            feeder.FR_FEEDER.setPosition(feederPos);
            // Telemetry lines
            telemetry.addData("back feeder pos", feeder.BL_FEEDER.getPosition());
            telemetry.addData("front feeder pos", feeder.FR_FEEDER.getPosition());
            telemetry.update();
        }
    }

    private void initialization() {
        feeder = new Feeder(hardwareMap);
    }
}
