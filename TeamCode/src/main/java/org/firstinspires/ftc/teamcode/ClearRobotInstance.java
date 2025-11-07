package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ClearRobotInstance extends LinearOpMode {
    public void runOpMode(){
        Robot.getInstance(null,null,null).clearInstance();
    }
}
