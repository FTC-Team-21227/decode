package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Intake Test")
// Program used to test hood positions
public class IntakeTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Intake hood;


    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) { // Lowest angle
                hood.intake();
            }
            if (gamepad1.b) { // Highest angle (41 degrees)
                hood.outtake();
            }
            if (gamepad1.x) { // Increase hood position
                hood.stop();
            }

            // Telemetry lines
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Intake(hardwareMap);
    }
}
