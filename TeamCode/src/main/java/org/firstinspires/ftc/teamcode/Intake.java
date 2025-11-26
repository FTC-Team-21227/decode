package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

public class Intake {
    //power
    DcMotor INTAKE;
    double power = 0;
    boolean paused = false;
    double voltageComp = 1;
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
        if (!paused) INTAKE.setPower(1*Robot.Constants.intakePower);
        power = 1*Robot.Constants.intakePower;
    }
    public void slowIntake(){
        if (!paused) INTAKE.setPower(1*Robot.Constants.slowIntakePower);
        power = 1*Robot.Constants.slowIntakePower;
    }
    public void outtake(){
        if (!paused) INTAKE.setPower(-1*Robot.Constants.outtakePower);
        power = -1*Robot.Constants.outtakePower;
    }
    public void strongOuttake(){
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
    public void shortOuttake(){
        if (!paused) INTAKE.setPower(-1*Robot.Constants.outtakePower);
        paused = true;
    }
    public void proceed() {
        INTAKE.setPower(power * voltageComp);
        paused = false;
    }
//    public void updateComp(){
//        voltageComp = 14.0/Flywheel.volts;
////        if (voltageComp %0.1==0){
//            RobotLog.d("comp"+voltageComp);
//            RobotLog.d("volts"+Flywheel.volts);
////        }
//    }
}
