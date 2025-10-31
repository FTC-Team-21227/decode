package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.MidpointTimer;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class TuneFeedforward extends OpMode {
    DcMotorEx FLYWHEEL;
    double POWER_PER_SEC = 0.1;
    double POWER_MAX = 1;
    MidpointTimer t;
    VoltageSensor v;

    public void init(){
        FLYWHEEL = hardwareMap.get(DcMotorEx.class, "flywheel");
        FLYWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        v = hardwareMap.get(VoltageSensor.class,"Control Hub");
//        FLYWHEEL.setVelocityPIDFCoefficients();
        t = new MidpointTimer();
        telemetry.update();
    }
    public void loop(){
        double time = t.seconds();
        double power = power(time);
        FLYWHEEL.setPower(power);
        double vel = FLYWHEEL.getVelocity();
        double bv = v.getVoltage();
        double vapp = power * bv;
        RobotLog.dd(""+vel,""+vapp);
        telemetry.addData("vel", vel);
        telemetry.addData("seconds", time);
        telemetry.addData("power", power);
        telemetry.addData("voltage applied", vapp);
        telemetry.addData("battery voltage", bv);
        telemetry.update();
    }
    public double power(double seconds){
        return Math.min(POWER_PER_SEC * seconds, POWER_MAX);
    }

}

