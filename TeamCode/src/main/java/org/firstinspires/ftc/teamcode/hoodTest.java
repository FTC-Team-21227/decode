package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Hood Test")
// Program used to test hood positions
public class hoodTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Hood hood;

    double HoodAngle=50;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) { // Lowest angle
                HoodAngle = 0;
            }
            if (gamepad1.yWasPressed()) { // Highest angle (41 degrees)
                HoodAngle = 35;
            }
            if (gamepad1.dpadUpWasPressed()) { // Increase hood position
                HoodAngle += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()) { // Decrease hood position
                HoodAngle -= 0.01;
            }
//            hood.turnToAngle(HoodAngle);
            hood.HOOD.setPosition(HoodAngle);
            // Telemetry lines
            telemetry.addData("Hood target angle", HoodAngle);
            telemetry.addData("Hood curr angle", hood.getAngle());
            telemetry.addData("Hood pos", hood.HOOD.getPosition());
//            telemetry.addData("outside range", hood.commandedOutsideRange());
            telemetry.update();
        }
    }

    private void initialization() {
        hood = new Hood(hardwareMap);
        HoodAngle = 0;
    }
}
