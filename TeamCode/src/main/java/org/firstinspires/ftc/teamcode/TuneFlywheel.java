package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp (name = "TuneFlywheel")
/**
 * 10/11/25: Runs the flywheel and prints its velocity
 */
public class TuneFlywheel extends LinearOpMode {
    Flywheel f;
    Voltage v;
    public static int targetVel = 0;
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        f = new Flywheel(hardwareMap);
        v = new Voltage(hardwareMap.get(VoltageSensor.class,"Control Hub"));
        waitForStart();
        while (opModeIsActive()){
            double k = 0;
            if (gamepad1.b){
                f.spinTo(0);
                f.FLYWHEEL.setPower(0);}
            else k = f.spinTo(targetVel);
            telemetry.addData("target v", targetVel);
            telemetry.addData("motor v", f.getVel());
            telemetry.addData("motor pid power", k);
            telemetry.update();
        }
    }
}
