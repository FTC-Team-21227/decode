package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Flywheel Test")
// Program used to test out flywheel speed based on robot distance and height difference from goal
public class shooterTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    private Servo HoodServo;
    private CRServo TurnTable;
    private DcMotor FlyWheelMotor;
    private DigitalChannel LED_DigitalChannel;

    // Editable in dashboard, distance and height difference from robot to goal
    public static double distance;
    public static double heightDiff;

    double flywheelVelocity;
    double HoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive()) {
            // Run flywheel based on calculated velocity
            flywheelVelocity = calcShooterVel(telemetry, distance, heightDiff);
            setFlyWheelSpeed(flywheelVelocity, gamepad1.a);
            // Telemetry lines
            telemetry.addData("Distance", distance);
            telemetry.addData("Height Diff", heightDiff);
            telemetry.addData("Flywheel Target Vel", flywheelVelocity);
            telemetry.addData("Flywheel Cur Vel", ((DcMotorEx) FlyWheelMotor).getVelocity());
            telemetry.update();

        }
    }

    /**
     * Calculates the velocity based on height difference from shooter to goal, and distance from shooter to goal base.
     * @return calculated velocity (double)
     */
    private double calcShooterVel(Telemetry telemetry, double d, double deltaH) {
        // Given d (horizontal distance) and deltaH (height difference)
        double g = 386.22; // in/s^2

        // Calculate launch angle theta
        double theta = Math.atan(7 * deltaH / (3 * d));

        // Calculate time of flight t_f
        double t_f = Math.sqrt(8 * deltaH / (3 * g));

        // Calculate initial speed v

//        // Convert v to RPM (example calibration: v = wheelCircumference * RPM / 60)
//        double wheelDiameter = 3.78; // inches, for example
//        double wheelCircumference = Math.PI * wheelDiameter;
//        double rpm = (v * 60) / wheelCircumference; // RPM

        return d / (t_f * Math.cos(theta)); // Velocity
    }

    private void initialization() {
        LED_DigitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
        LED_DigitalChannel.setState(true);
        FlyWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        FlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ((DcMotorEx) FlyWheelMotor).setVelocity(0);
        HoodServo.setDirection(Servo.Direction.FORWARD);
        HoodServo.scaleRange(0.27, 1);
        HoodPosition = 0;
        HoodServo.setPosition(HoodPosition);
        TurnTable.setDirection(CRServo.Direction.FORWARD);
        TurnTable.setPower(0);
    }

    /**
     * Set flywheel to velocity (double)
     * @param velocity velocity to set flywheel to
     * @param shotRequested gamepad button to shoot
     */
    private void setFlyWheelSpeed(double velocity, boolean shotRequested) {
        if (shotRequested) {((DcMotorEx) FlyWheelMotor).setVelocity(velocity);}
    }
}
