package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Flywheel {
    //pid methods
    DcMotorEx FLYWHEEL;
    double targetVel;
    //power
    public Flywheel (HardwareMap hardwareMap){
        FLYWHEEL = hardwareMap.get(DcMotorEx.class, "flywheel");
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Robot.Constants.p,Robot.Constants.i,Robot.Constants.d,Robot.Constants.f));
    }
    public void spinTo(double vel){
        targetVel = vel;
        FLYWHEEL.setVelocity(targetVel,AngleUnit.RADIANS);
    }
    public double getVel(){
        return FLYWHEEL.getVelocity(AngleUnit.RADIANS);
    }
}
