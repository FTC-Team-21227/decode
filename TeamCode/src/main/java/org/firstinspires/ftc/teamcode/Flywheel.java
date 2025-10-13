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
    public void spinTo(double vel) {
//        if (configvalues.p != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p || configvalues.f != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f)
//            FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(configvalues.p, configvalues.i, configvalues.d, configvalues.f));

        targetVel = vel;
        FLYWHEEL.setVelocity(targetVel);
    }
    public double getVel(){
        return FLYWHEEL.getVelocity();
    }
}
