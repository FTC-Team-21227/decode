package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Flywheel {
    //pid methods
    CachedMotor FLYWHEEL;
    double targetVel;
    double lastPower;
    PIDController pid;
    FeedforwardController f;
    VoltageSensor v;
    //power
    public Flywheel (HardwareMap hardwareMap){
//        FLYWHEEL = hardwareMap.get(DcMotorEx.class, "flywheel");
        FLYWHEEL = new CachedMotor(hardwareMap.get(DcMotorEx.class,"flywheel"));
        FLYWHEEL.setDirection(DcMotorSimple.Direction.REVERSE);
        FLYWHEEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLYWHEEL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                FLYWHEEL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        FLYWHEEL.getMotor().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,Robot.Constants.kI,Robot.Constants.kD,Robot.Constants.kF));
        v = hardwareMap.get(VoltageSensor.class, "Control Hub");
        pid = new PIDController(Robot.Constants.kP,Robot.Constants.kI,Robot.Constants.kD);
        f = new FeedforwardController(Robot.Constants.kS,Robot.Constants.kV);
    }
    /**
     * @param vel Velocity the flywheel will spin at
     */
    public double spinTo(double vel) {
//        if (configvalues.p != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p || configvalues.f != FLYWHEEL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f)
//            FLYWHEEL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(configvalues.p, configvalues.i, configvalues.d, configvalues.f));
        targetVel = vel;
        double curVel = getVel();
        //pidf bangbang: go as fast as possible when spinning up then slow down for the last hundred ticks/s
        if (/*Math.abs(targetVel-curVel) > 100*/ true) {
            double pdds = pid.calculate(curVel, targetVel);
//        FLYWHEEL.getMotor().setVelocity(targetVel);
            FLYWHEEL.setPower((pdds + f.calculate(targetVel)) / v.getVoltage());
            return pdds + f.calculate(targetVel);
        }
        else{
            double power = Math.signum(targetVel-curVel);
            FLYWHEEL.setPower(power);
            return power;
        }
    }
    public void setPower(double power){
        if (power != lastPower){
            lastPower = power;
            FLYWHEEL.setPower(power);
        }
    }
    public double getVel(){
        return FLYWHEEL.getMotor().getVelocity();
    }
}
