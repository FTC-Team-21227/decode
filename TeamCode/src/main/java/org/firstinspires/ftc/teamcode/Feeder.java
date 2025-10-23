package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
   Servo FR_FEEDER, BL_FEEDER;

    public Feeder (HardwareMap hardwareMap){
        FR_FEEDER = hardwareMap.get(Servo.class, "fr_feeder"); // Elevator
        BL_FEEDER = hardwareMap.get(Servo.class, "bl_feeder");
//        FR_FEEDER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FR_FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FR_FEEDER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_FEEDER.setDirection(Servo.Direction.FORWARD);
        FR_FEEDER.scaleRange(Robot.Constants.feederScale0, Robot.Constants.feederScale1); // left = low, right = high
        BL_FEEDER.setDirection(Servo.Direction.FORWARD);
        BL_FEEDER.scaleRange(Robot.Constants.feederScale0, Robot.Constants.feederScale1);
    }
    public void up(Servo feeder){
        feeder.setPosition(0.7);
    }
    public void down(Servo feeder){
        feeder.setPosition(0);
    }
}
