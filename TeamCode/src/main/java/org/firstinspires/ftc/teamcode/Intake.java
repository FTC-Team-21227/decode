package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //power
    DcMotor INTAKE;
    double power = 0;
    boolean paused = false;
    //power
    public Intake (HardwareMap hardwareMap){
        INTAKE = hardwareMap.get(DcMotor.class, "intake");
        INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        INTAKE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // Modes
    public void intake(){
        if (!paused) INTAKE.setPower(1);
        power = 1;
    }
    public void outtake(){
        if (!paused) INTAKE.setPower(-1);
        power = -1;
    }
    public void stop(){
        if (!paused) INTAKE.setPower(0);
        power = 0;
    }
    // Start and stop
    public void pause(){
        INTAKE.setPower(0);
        paused = true;
    }
    public void proceed() {
        INTAKE.setPower(power);
        paused = false;
    }
}
