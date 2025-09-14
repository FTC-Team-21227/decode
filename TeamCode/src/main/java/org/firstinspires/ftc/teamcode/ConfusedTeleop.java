package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp
public class ConfusedTeleop extends OpMode {
    DcMotorEx fly;
    public void init(){
        fly = hardwareMap.get(DcMotorEx.class, "flywheel");
        fly.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
    boolean a = true;
    public void loop(){
        if (a) {fly.setVelocity (100); a = false;}
        telemetry.addData("velocity", fly.getVelocity());
        telemetry.update();
    }
}
