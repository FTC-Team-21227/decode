package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Flywheel {
    //pid methods
    DcMotorEx FLYWHEEL;
    double targetVel;
    PIDController pid;
    FeedforwardController f;
    //power
    public Flywheel (HardwareMap hardwareMap){
        FLYWHEEL = hardwareMap.get(DcMotorEx.class, "flywheel");
        FLYWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Robot.Constants.kP,Robot.Constants.kI,Robot.Constants.kD,Robot.Constants.kF));
        pid = new PIDController(Robot.Constants.kP,Robot.Constants.kI,Robot.Constants.kD);
        f = new FeedforwardController(Robot.Constants.kS,Robot.Constants.kV);
    }
    /**
     * @param vel Velocity the flywheel will spin at
     */
    public void spinTo(double vel/*, double volts*/) {
//        if (configvalues.p != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p || configvalues.f != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f)
//            FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(configvalues.p, configvalues.i, configvalues.d, configvalues.f));

        targetVel = vel;
        FLYWHEEL.setVelocity(targetVel);
//        FLYWHEEL.setPower((pid.calculate(FLYWHEEL.getVelocity(), targetVel) + f.calculate(targetVel))/volts);
    }
    public double getVel(){
        return FLYWHEEL.getVelocity();
    }
}
