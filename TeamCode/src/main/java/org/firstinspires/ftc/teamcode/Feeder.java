package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder {
   Servo FEEDER;
    //power
    public Feeder (HardwareMap hardwareMap){
        FEEDER = hardwareMap.get(Servo.class, "feeder");
//        FEEDER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FEEDER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void up(){
        FEEDER.setPosition(1);

    }
    public void down(){
        FEEDER.setPosition(0);
    }
}
