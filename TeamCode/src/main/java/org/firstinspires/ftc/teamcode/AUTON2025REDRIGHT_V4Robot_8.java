package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.autons;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;
//
//@Autonomous(name = "REDRIGHT_6spec_CV")
//public class AUTON2025REDRIGHT_V4Robot_8 extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        PipeCamera cam = new PipeCamera(hardwareMap,telemetry,true,"red", false);
//
//        ARM1_V3Robot arm1 = new ARM1_V3Robot(hardwareMap);
//        ARM2_V3Robot arm2 = new ARM2_V3Robot(hardwareMap);
//        CLAW_NEW claw = new CLAW_NEW(hardwareMap);
//        INTAKE_ANGLE_NEW intake_angle = new INTAKE_ANGLE_NEW(hardwareMap);
//        CLAW_ANGLE_NEW claw_angle = new CLAW_ANGLE_NEW(hardwareMap);
//        SWEEPER sweeper = new SWEEPER(hardwareMap);
//        double firstSpecDistance = -51;
//        double otherSpecDistance = -35.5;
//        double wallGrab = -46.5;
//        double wallGrab1 = 21.5;
//        double wallGrabAngle = -85;
//        double frictionConstant = 0;
////        pushing timing to the limits
//
//        String camoora = "";
//        while (camoora.equals("") && !isStopRequested()){
//            if (gamepad1.left_trigger > 0.1) camoora = "a";
//            if (gamepad1.right_trigger > 0.1) camoora = "b";
//            telemetry.addLine("Use Camera? (RT = yes, LT = no)");
//            telemetry.update();
//        }
//        if (camoora.equals("a")) cam.off();
//        double pose_X = 10.5;
//        double X = 0;
//        boolean cont = true;
//        boolean a = false;
//        boolean b= false;
//        boolean x= false;
//        boolean y= false;
//        boolean up= false;
//        boolean down= false;
//        boolean left= false;
//        boolean right= false;
//        boolean RB= false;
//        boolean LB= false;
//        boolean back= false;
//        boolean decimal = false;
//        boolean right_stick_button = false;
//        boolean start= false;
//        boolean LS = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    X += 0.1;
//                }
//                else {
//                    X = 10 * X + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    X += 0.2;
//                }
//                else {X = 10*X+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    X += 0.3;
//                }
//                else {X = 10*X+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    X += 0.4;
//                }
//                else {X = 10*X+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    X += 0.5;
//                }
//                else {X = 10*X+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    X += 0.6;
//                }
//                else {X = 10*X+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    X += 0.7;
//                }
//                else {X = 10*X+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    X += 0.8;
//                }
//                else {X = 10*X+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    X += 0.9;
//                }
//                else {X = 10*X+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    X += 0.0;
//                }
//                else {X = 10*X+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.left_stick_button && !LS){
//                X *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                X = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                if (X!=0)pose_X = X;
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pose_X ",pose_X);
//            telemetry.addData("Pos X ",X);
//            telemetry.addLine("Note: x < 5 will make it go left" );
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative,  start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//        double pose_Y = 3;
//        double Y = 0;
//        cont = true;
//        back = false;
//        while (cont && !isStopRequested()){
//            if (gamepad1.a && !a){
//                if (decimal){
//                    Y += 0.1;
//                }
//                else {
//                    Y = 10 * Y + 1;
//                }
//            }
//            a = gamepad1.a;
//            if (gamepad1.b && !b){
//                if (decimal){
//                    Y += 0.2;
//                }
//                else {Y = 10*Y+2;}
//            }
//            b = gamepad1.b;
//            if (gamepad1.x && !x){
//                if (decimal){
//                    Y += 0.3;
//                }
//                else {Y = 10*Y+3;}
//            }
//            x = gamepad1.x;
//            if (gamepad1.y && !y){
//                if (decimal){
//                    Y += 0.4;
//                }
//                else {Y = 10*Y+4;}
//            }
//            y = gamepad1.y;
//            if (gamepad1.dpad_up && !up){
//                if (decimal){
//                    Y += 0.5;
//                }
//                else {Y = 10*Y+5;}
//            }
//            up = gamepad1.dpad_up;
//            if (gamepad1.dpad_down && !down){
//                if (decimal){
//                    Y += 0.6;
//                }
//                else {Y = 10*Y+6;}
//            }
//            down = gamepad1.dpad_down;
//            if (gamepad1.dpad_left && !left){
//                if (decimal){
//                    Y += 0.7;
//                }
//                else {Y = 10*Y+7;}
//            }
//            left = gamepad1.dpad_left;
//            if (gamepad1.dpad_right && !right){
//                if (decimal){
//                    Y += 0.8;
//                }
//                else {Y = 10*Y+8;}
//            }
//            right = gamepad1.dpad_right;
//            if (gamepad1.right_bumper && !RB){
//                if (decimal){
//                    Y += 0.9;
//                }
//                else {Y = 10*Y+9;}
//            }
//            RB = gamepad1.right_bumper;
//            if (gamepad1.left_bumper && !LB){
//                if (decimal){
//                    Y += 0.0;
//                }
//                else {Y = 10*Y+0;}
//            }
//            LB = gamepad1.left_bumper;
//            if (gamepad1.back && !back){
//                decimal = true;
//            }
//            back = gamepad1.back;
//            if (gamepad1.left_stick_button && !LS){
//                Y *= -1;
//            }
//            LS = gamepad1.left_stick_button;
//            if (gamepad1.right_stick_button && !right_stick_button){
//                Y = 0;
//                decimal = false;
//            }
//            right_stick_button = gamepad1.right_stick_button;
//            if (gamepad1.start && !start){
//                if (Y!=0)pose_Y = Y;
//                cont = false;
//            }
//            start = gamepad1.start;
//            telemetry.addData("Pose Y ",pose_Y);
//            telemetry.addData("Pos Y ",Y);
//            if (decimal){
//                telemetry.addData("In decimal mode ", "only 1 decimal place permitted");
//            }
//            telemetry.addLine("a=1, b=2, x=3, y=4, up=5, down=6, left=7, right=8, RB=9, LB=0, back=decimal, Left Stick Button = negative, start = continue, Right Stick Button = erase");
//            telemetry.update();
//        }
//
//        telemetry.addData("Pos X ",pose_X);
//        telemetry.addData("Pos Y ",pose_Y);
//        telemetry.addLine("If incorrect, stop and reinit");
//        telemetry.update();
//        PoseStorage.grabColorPose = new Pose2d(pose_X,-47+pose_Y,Math.toRadians(90));
//
//        Vector2d firstPlace = new Vector2d(10.5,firstSpecDistance);
//        if (pose_X < 5.0) firstPlace = new Vector2d(0.0, firstSpecDistance+4);
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
////                .setTangent(Math.toRadians(90))
//<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_8.java
//                .waitSeconds(0.4)
////                .splineToConstantHeading(new Vector2d(pose_X,firstSpecDistance),Math.toRadians(90));
//                .strafeTo(firstPlace);
//========
//                .waitSeconds(0.3)
//                .strafeTo(new Vector2d(pose_X,firstSpecDistance));
//
//>>>>>>>> 58bb715160d41ba8101731794519fc073136a26c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_5.java
//
//        MecanumDrive.PARAMS.axialGain = 2.0;
//        MecanumDrive.PARAMS.lateralGain = 1.0;
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(69, -44.5, Math.toRadians(-90))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.1)
////                .splineToConstantHeading(new Vector2d(7.5,otherSpecDistance),Math.toRadians(90));
//                .splineToLinearHeading(new Pose2d(5.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
//                .splineToConstantHeading(new Vector2d(5.5,otherSpecDistance-3),Math.toRadians(-45))
////                .splineToConstantHeading(new Vector2d(40,-48.25),Math.toRadians(-12));
//                .splineToSplineHeading(new Pose2d(40,wallGrab,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(40, wallGrab,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.1)
////                .splineToConstantHeading(new Vector2d(8.5,otherSpecDistance),Math.toRadians(90));
//                .splineToLinearHeading(new Pose2d(7.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(7.5, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(10));
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//                .splineToConstantHeading(new Vector2d(7.5,otherSpecDistance-3),Math.toRadians(-45))
////                .splineToConstantHeading(new Vector2d(40,-48),Math.toRadians(-12));
//                .splineToSplineHeading(new Pose2d(40,wallGrab+0.3,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
//        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(40, wallGrab+0.3,Math.toRadians(wallGrabAngle))) //pick up and place second specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.1)
//                .splineToLinearHeading(new Pose2d(10,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(10, otherSpecDistance, Math.toRadians(-90))) //go to third specimen
//                .setTangent(Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(40,-40),Math.toRadians(0))
////                .strafeTo(new Vector2d(40,-45.5),new TranslationalVelConstraint(30));
//                .splineToConstantHeading(new Vector2d(10,otherSpecDistance-3),Math.toRadians(-45))
////                .splineToConstantHeading(new Vector2d(40,-48),Math.toRadians(-12));
//                .splineToSplineHeading(new Pose2d(40,wallGrab+0.6,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
//        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(40, wallGrab+0.6,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.1)
//                .splineToLinearHeading(new Pose2d(12.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(12.5, otherSpecDistance, Math.toRadians(-90))) //park
//                .setTangent(Math.toRadians(-90))
////                .strafeTo(new Vector2d(30,-58));
//                .splineToConstantHeading(new Vector2d(12.5,otherSpecDistance-3),Math.toRadians(-45))
//                .splineToSplineHeading(new Pose2d(40,wallGrab+0.9,Math.toRadians(wallGrabAngle)),Math.toRadians(-12));
//        TrajectoryActionBuilder tab14 = drive.actionBuilder(new Pose2d(40, wallGrab+0.9,Math.toRadians(wallGrabAngle))) //pick up and place fourth specimen
//                .setTangent(Math.toRadians(170))
//                .waitSeconds(0.1)
//                .splineToLinearHeading(new Pose2d(14.5,otherSpecDistance,Math.toRadians(-90)),Math.toRadians(90));
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        claw.closeClaw(),
//                        intake_angle.RotatePosition1(),
//                        claw_angle.forward()
//                )
//        );
//
//        Action firstTrajectory = tab1.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();
//        Action tenthTrajectory = tab10.build();
//        Action eleventhTrajectory = tab11.build();
//        Action twelfthTrajectory = tab12.build();
//        Action thirteenthTrajectory = tab13.build();
//        Action fourteenthTrajectory = tab14.build();
//
//        waitForStart();
//
//        Actions.runBlocking(
//                new SequentialAction(
//            //first specimen
//            new ParallelAction(
//                claw.closeClaw(),
//                intake_angle.RotatePosition3(),
//                arm1.liftRung_First(0,1.15),
//                arm2.liftRung2_First(0,1.15),
//                firstTrajectory,
//                claw.openClawMore(1.15)
//            ),
//            cam.comp(0)
//            )
//        );
//        Pose2d color_pose = PoseStorage.grabColorPose;
//        if (ExcludePipeline.printStuff) RobotLog.dd("COLOR_POSE", color_pose.position.x + ", " + color_pose.position.y + ", " + color_pose.heading.toDouble());
//        MecanumDrive.PARAMS.axialGain = 8.0;
//        MecanumDrive.PARAMS.lateralGain = 8.0;
//        Action thirdTrajectory = drive.actionBuilder(new Pose2d(firstPlace, Math.toRadians(90))) //push colored samples
//                .strafeTo(new Vector2d(firstPlace.x, firstSpecDistance-5))
//                .strafeTo(color_pose.position)
//                .build();
//        Action secondTrajectory = drive.actionBuilder(color_pose) //push colored samples
//                .setTangent(Math.toRadians(-90))
//                .waitSeconds(0.3)
//                .splineToConstantHeading(new Vector2d(pose_X,-48),Math.toRadians(-90))
////                .splineToConstantHeading(new Vector2d(15,-54),Math.toRadians(-45))
//                .splineToSplineHeading(new Pose2d(41+frictionConstant, -52,Math.toRadians(-30)),Math.toRadians(0))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(41+frictionConstant,-22,Math.toRadians(-90)),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(48.5+frictionConstant, -14),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(51.5, -22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(51.5, -46),Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(51.5, -22),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(58.5+frictionConstant, -16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(61.5+frictionConstant, -22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(61.5, -46),Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(61.5, -22),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(66+frictionConstant, -16),Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(69+frictionConstant, -22),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(69, -35.5),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(69, -46),Math.toRadians(-90), new TranslationalVelConstraint(15))
//                .build();
//        Actions.runBlocking(
//                new SequentialAction(
//            new ParallelAction(
////                                claw.openClawMore(),
//<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_8.java
//                        cam.close(0),
//                        thirdTrajectory,
//                        intake_angle.RotatePosition0_left(0.5),
//                        arm1.liftVertFloor(0.2,1.1),
//                        arm2.liftVertFloor(0.2,1.1)
//            ),
//            new ParallelAction(
//                intake_angle.RotatePosition2(),
//                claw.closeClaw(0.4)
//            )
//        )
////                )
//========
//                            cam.close(0),
//                                thirdTrajectory,
//                                intake_angle.RotatePosition0_left(0.5),
//                                arm1.liftVertFloor(0.2,1.1),
//                                arm2.liftVertFloor(0.2,1.1),
//                                intake_angle.RotatePosition2(1.5),
//                                claw.closeClaw(2)
//                        )
//                )
//>>>>>>>> 58bb715160d41ba8101731794519fc073136a26c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_5.java
//        );
//        MecanumDrive.PARAMS.axialGain = 2.0;
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                secondTrajectory,
//<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_8.java
//                                intake_angle.RotatePosition0(0.3),
//                                claw.openClaw(1.6 /*1.2*/),
//                                arm1.liftDown(2),
//                                arm2.liftDown(2),
//                                arm1.liftWall2_First(7,1.4),
//                                arm2.liftWall2(7,1.4),
//                                claw_angle.backward(7),
//                                intake_angle.RotatePositionNegative1(7.7),
//                                sweeper.RotatePosition0(9)
//========
//                                intake_angle.RotatePosition0_left(0.3),
//                                arm1.liftWall_First(1,1.4),
//                                arm2.liftWall2_First(1,1.4),
//                                intake_angle.RotatePositionNegative2(1.0),
//                                claw.openClaw(1.4)
//>>>>>>>> 58bb715160d41ba8101731794519fc073136a26c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_5.java
//                        ),
//                        claw.closeClaw()
//                )
//        );
////        Action fifthTrajectory = drive.actionBuilder(drive.localizer.getPose()) //push colored samples
////                .strafeToLinearHeading(new Vector2d(50.5+frictionConstant, -53),Math.toRadians(90))
////                .build();
////        Actions.runBlocking(
////                new SequentialAction(
////                        new ParallelAction(
////                                intake_angle.RotatePosition0_left(),
////                                fifthTrajectory,
////                                arm1.liftFloor(0, 1.3),
////                                arm2.liftFloor(0, 1.3),
////                                claw.closeClaw(1)
////                        ),
////                        new ParallelAction(
////                                claw.openClaw(1.2),
////                                fourthTrajectory,
////                                arm1.liftWall_First(0,1.4),
////                                arm2.liftWall2_First(0,1.4),
////                                intake_angle.RotatePositionNegative2(0.5)
////                        ),
////                        claw.closeClaw()
////                )
////        );
//        MecanumDrive.PARAMS.lateralGain = 1.0;
//        MecanumDrive.PARAMS.headingGain = 2.0;
//        Actions.runBlocking(
//                new SequentialAction(
//                        //pick up and place second specimen
//                        new ParallelAction(
//                                sweeper.RotatePosition1(0),
//                                arm1.liftRung2(0.3,2.0, false),
//                                arm2.liftRung2(0.3, 2.0, false),
//                                sixthTrajectory,
//                                claw_angle.forward(0.5),
//                                intake_angle.RotatePosition0(0.4)
//                        ),
//                        //go to third specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_8.java
//                                seventhTrajectory,
//                                arm1.liftWall2(0.4,1.3), //0.5
//                                arm2.liftWall2(0.4,1.3),
//========
//                                intake_angle.RotatePositionNegative1(0),
//                                arm1.liftFloor(0.3,1.1), //0.5
//                                arm2.liftFloor(0.3,1.1),
//>>>>>>>> 58bb715160d41ba8101731794519fc073136a26c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_5.java
//                                claw_angle.backward(0),
//                                intake_angle.RotatePositionNegative1(0.6),
//                                claw.closeClaw(1.5)
//                        ),
//                        //pick up and place third specimen
//                        new ParallelAction(
//                                arm1.liftRung2(0.1,1.5, false),
//                                arm2.liftRung2(0.1,1.5, false),
//                                eighthTrajectory,
//                                claw_angle.forward(0.3),
//                                intake_angle.RotatePosition0(0.2)
//                        ),
//                        //go to fourth specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                ninthTrajectory,
//                                arm1.liftWall2(0.4,1.3),
//                                arm2.liftWall2(0.4,1.3),
//                                claw_angle.backward(0),
//                                intake_angle.RotatePositionNegative1(0.6),
//                                claw.closeClaw(1.5)
//                        ),
//                        //pick up and place fourth specimen
//                        new ParallelAction(
//                                arm1.liftRung2(0.1,1.5, false),
//                                arm2.liftRung2(0.1,1.5, false),
//                                tenthTrajectory,
//                                claw_angle.forward(0.3),
//                                intake_angle.RotatePosition0(0.2)
//                        ),
//                        //go to fifth specimen
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                eleventhTrajectory,
//                                arm1.liftWall2(0.4,1.3),
//                                arm2.liftWall2(0.4,1.3),
//                                claw_angle.backward(0),
//                                intake_angle.RotatePositionNegative1(0.6),
//                                claw.closeClaw(1.5)
//                        ),
//                        //pick up and place fifth specimen
//                        new ParallelAction(
//                                arm1.liftRung2(0.1,1.5, false),
//                                arm2.liftRung2(0.1,1.5, false),
//                                twelfthTrajectory,
//                                claw_angle.forward(0.3),
//                                intake_angle.RotatePosition0(0.2)
//                        ),
//                        new ParallelAction(
//                                claw.openClawMore(),
//                                thirteenthTrajectory,
//                                arm1.liftWall2(0.4,1.3),
//                                arm2.liftWall2(0.4,1.3),
//                                claw_angle.backward(0),
//                                intake_angle.RotatePositionNegative1(0.6),
//                                claw.closeClaw(1.5)
//                        ),
//                        new ParallelAction(
//                                arm1.liftRung2(0.1,1.5, false),
//                                arm2.liftRung2(0.1,1.5, false),
//                                fourteenthTrajectory,
//                                claw_angle.forward(0.3),
//                                intake_angle.RotatePosition0(0.2)
//                        ),
//                        claw.openClaw(),
//                        arm1.liftHighBasket(1000)
//                )
//        );
//        Pose2d pose = drive.localizer.getPose();
//        PoseStorage.currentPose =
//        new Pose2d(
//<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_8.java
//                pose.position,
//========
//                pose.position.x + 15*Math.sqrt(2),
//                pose.position.y + 15*Math.sqrt(2),
//>>>>>>>> 58bb715160d41ba8101731794519fc073136a26c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autons/AUTON2025REDRIGHT_V4Robot_5.java
//                pose.heading.toDouble()-Math.toRadians(90)
//        );
//    }
//}
