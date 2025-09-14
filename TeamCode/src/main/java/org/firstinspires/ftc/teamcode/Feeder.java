package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    DcMotor FEEDER;
    //power
    public Feeder (HardwareMap hardwareMap){
        FEEDER = hardwareMap.get(DcMotor.class, "feeder");
        FEEDER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FEEDER.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void rollIn(){
        FEEDER.setPower(Robot.Constants.feederPower);
    }
    public void stop(){
        FEEDER.setPower(0);
    }
}
