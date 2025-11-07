package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
   CachedServo FR_FEEDER, BL_FEEDER;

    public Feeder (HardwareMap hardwareMap){
//        FR_FEEDER = hardwareMap.get(Servo.class, "fr_feeder");/* // Elevator
//        BL_FEEDER = hardwareMap.get(Servo.class, "bl_feeder");*/
        FR_FEEDER = new CachedServo(hardwareMap.get(Servo.class, "fr_feeder")); // Elevator
        BL_FEEDER = new CachedServo(hardwareMap.get(Servo.class, "bl_feeder"));
        FR_FEEDER.setDirection(Servo.Direction.REVERSE);
        FR_FEEDER.scaleRange(Robot.Constants.feederScale0, Robot.Constants.feederScale1); // left = low, right = high
        BL_FEEDER.setDirection(Servo.Direction.FORWARD);
        BL_FEEDER.scaleRange(Robot.Constants.feederScale0, Robot.Constants.feederScale1);
    }
    public void up(CachedServo feeder){
        feeder.setPosition(0.7);
    }
    public void down(CachedServo feeder){
        feeder.setPosition(0);
    }

    public void upFR() { FR_FEEDER.setPosition(0.85); }
    public void downFR() { FR_FEEDER.setPosition(0.44); }

    public void upBL() { BL_FEEDER.setPosition(0.98); }
    public void downBL() { BL_FEEDER.setPosition(0.57); } // 0.6

}
