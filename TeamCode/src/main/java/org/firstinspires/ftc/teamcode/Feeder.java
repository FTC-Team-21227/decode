package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    DcMotor feeder;
    //power
    public Feeder (HardwareMap hardwareMap){
        feeder = hardwareMap.get(DcMotor.class, "feeder");
    }
    public void rollIn(){
        feeder.setPower(1);
    }
}
