package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class Teleop_Smooth extends OpMode {
    Robot robot;
    // Shooter booleans
    boolean intake = false; // False = stop, true = intake
    boolean setPose = false; // Lock pose for shooter to manually adjust
    boolean human = false; // Human feed
    boolean moveShot = false;
    boolean disableFlywheel = false;
    // Drivetrain booleans
    boolean slow = false;
    boolean p2p = false;
    boolean RT = false;
    boolean LT = false;
    TelemetryPacket packet = new TelemetryPacket();

    public void init(){
        robot = Robot.getInstance(new Pose2d(0,0, Math.PI), Robot.Color.BLUE); //start facing the goals, RED poses
        robot.initTeleop(hardwareMap, telemetry);
        telemetry.update();
    }

    public void start(){
        robot.startTeleop();
    }

    public void loop(){
        // Toggles: LB-intake, x-setPose, y-human feed, leftStickButton-slow mode, a-moveShot, start-flywheel
        if (gamepad1.leftBumperWasPressed()) intake = !intake;
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) setPose = !setPose;
        if (gamepad2.yWasPressed()) human = !human;
        if (gamepad2.leftStickButtonWasPressed() /*|| gamepad1.leftStickButtonWasPressed()*/) slow = !slow;
//        if (gamepad1.yWasPressed()) p2p = !p2p;
        if (gamepad2.aWasPressed()) moveShot = !moveShot;
        if (gamepad1.startWasPressed() || gamepad2.startWasPressed()) disableFlywheel = !disableFlywheel;

        RT = gamepad2.right_trigger > 0.1;
        LT = gamepad2.left_trigger > 0.1;
//        robot.updateVoltage(telemetry);

        // Touchpad relocalizes robot aligned with goal post at the start of teleop
        robot.updateLocalizer(false, gamepad1.touchpadWasPressed(), telemetry);

        // Final version will be: LB = intake toggle, b = reverse, a = brief reverse, inslow needed?
        robot.controlIntake(intake, gamepad1.b, !intake, gamepad1.aWasPressed(), false, gamepad1.y);
        robot.setGoalTarget();

        /**
         * SHOOTER CONTROL Final version will be:
         * GAMEPAD 1:------------------------------------
         * RT = front feeder, LT = back feeder, RB = alternating feeder,
         * x = toggle setPose => shooter lock/manual control,
         * rightstick up/down = flywheel scale,
         * dpad up/down= hood, dpad left/right = turret,
         * GAMEPAD 2:------------------------------------
         * y = human feed toggle, start = power flywheel off,
         * bumpers and triggers = feeders manual control,
         * a = velocity correction
         */
        robot.updateShooter(gamepad1.right_trigger > 0.1,
                gamepad1.left_trigger > 0.1, gamepad1.right_bumper, telemetry,
                setPose, null,
                gamepad1.right_stick_y + gamepad2.right_stick_y,
                gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed(),
                gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed(),
                gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed(),
                gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed(),
                human, disableFlywheel, gamepad2.rightBumperWasPressed(),
                gamepad2.right_trigger > 0.1 && !RT, gamepad2.leftBumperWasPressed(),
                gamepad2.left_trigger > 0.1 && !LT, moveShot);

        // Final: toggle left stick button = slow mode, toggle y = p2p drive but it stops on its own
        p2p = robot.driveFieldCentric_Smooth(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, slow, p2p);

        // Display and telemetry
        robot.drive2.drawPose(packet);
        /*
        telemetry.addLine(""+robot.turret.turret.getPosition());
        telemetry.addLine(""+robot.drive2.localizer.getPose().position.x + ", "+robot.drive2.localizer.getPose().position.y + ", "+robot.drive2.localizer.getPose().heading.toDouble());
        telemetry.addLine(""+robot.hood.HOOD.getPosition());
        telemetry.addLine(""+robot.flywheel.FLYWHEEL.getMotor().getDirection());
        telemetry.addLine(""+robot.feeder.BL_FEEDER.getPosition());
        */
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void stop(){
        robot.setPose(robot.drive2.localizer.getPose());
    } // Update txWorldPinpoint as robot's current pose
}
