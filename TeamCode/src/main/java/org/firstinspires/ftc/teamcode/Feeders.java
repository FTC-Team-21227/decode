package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeders {
    CRServo left;
    CRServo right;
    public Feeders(HardwareMap hardwareMap){
        left = hardwareMap.get(CRServo.class, "feeder1");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right = hardwareMap.get(CRServo.class, "feeder2");
    }
    public void rollIn(){
        left.setPower(Robot.Constants.feederPower);
        right.setPower(Robot.Constants.feederPower);
    }
    public void stop(){
        left.setPower(0);
        right.setPower(0);
    }
}
