/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag Localization", group = "Concept")
//@Disabled
public class AprilTagLocalization2 {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    ElapsedTime pipelineTimer = new ElapsedTime();
    double averagePipe = 0;
    int successfulDetections = 0;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.25, 12, 0); // UNKNOWN CONSTANTS
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0); // UNKNOWN CONSTANTS

    public Position getCamPos()
    {
        return cameraPosition;
    }

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag; // analyzes frames coming from camera, attached to EOCV pipeline

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal; //manages the overall pipeline
//    OpenCvWebcam camera;

//    @Override
    public AprilTagLocalization2(HardwareMap hardwareMap){ // Running...

        initAprilTag(hardwareMap); // initialize AprilTag Processor
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Wait for the DS start button to be touched.

//        waitForStart();

//        while (opModeIsActive()) { // while the AprilTagLocalization is running
//
//            telemetryAprilTag();
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//            // Push telemetry to the Driver Station.
//            telemetry.update();
//            FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
////            camera.openCameraDevice();
////            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            if (!currentDetections.isEmpty())
//            {
//                TelemetryPacket packet = new TelemetryPacket();
////                packet.put();
//            }
//
//            // Save CPU resources; can resume streaming when needed.
//            if (gamepad1.dpad_down) {
//                visionPortal.stopStreaming();
//            } else if (gamepad1.dpad_up) {
//                visionPortal.resumeStreaming();
//            }
//
//            // Share the CPU.
//            sleep(20);
//        }

        // Save more CPU resources when camera is no longer needed.
//        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */

    private int getID(AprilTagDetection tag)
    {
        return tag.id;
    }

//    public double getYaw()
//    {
//        initAprilTag();
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
////        currentDetections.sort((a, b) -> Double.compare(b.ftcPose.range, a.ftcPose.range));
//        AprilTagDetection cur_tag = currentDetections.get(0);
//        return cur_tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//    }

    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(281.5573273, 281.366942, 156.3332591, 119.8965271)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(320,240));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 10);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public double telemetryAprilTag(Telemetry telemetry
    ) {
        telemetry.addData("FPS", visionPortal.getFps());
        pipelineTimer.reset();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine("robotPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addLine("FtcPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw));
                    telemetry.addLine("rawPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.rawPose.x,
                            detection.rawPose.y,
                            detection.rawPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)"));
                    double time = pipelineTimer.milliseconds();
                    averagePipe = (averagePipe*successfulDetections + time)/(successfulDetections+1);
                    successfulDetections++;
                    telemetry.addLine("\nPIPELINE TIME DELAY: (ms) " + time + "\n");
                    telemetry.addLine("\nAVERAGE DELAY: (ms) " + averagePipe + "\n");
                    telemetry.update();
                    return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.update();

        if (!currentDetections.isEmpty())
        {
            AprilTagDetection detection = currentDetections.get(0);
            return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return Double.NaN;
    }   // end method telemetryAprilTag()
    //Update the Pose of the TURRET CENTER relative to the field in Official FTC Global Field Coordinates using the Goal Apriltag.
    //Future improvement: Only use the correct goal color apriltag to localize.
    public Pose2d update(Telemetry telemetry
    ) {
        telemetry.addData("FPS", visionPortal.getFps()); //the FPS processed through the visionPortal for apriltag processing etc. =/= Dashboard Stream FPS
        pipelineTimer.reset();
        //Get all Apriltag detections from the apriltag processor of the visionPortal
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine("robotPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addLine("FtcPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw));
                    telemetry.addLine("rawPose");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.rawPose.x,
                            detection.rawPose.y,
                            detection.rawPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)"));
                    double time = pipelineTimer.milliseconds();
                    averagePipe = (averagePipe*successfulDetections + time)/(successfulDetections+1);
                    successfulDetections++;
                    //display the instantaneous delay and average delay.
                    telemetry.addLine("\nPIPELINE TIME DELAY: (ms) " + time + "\n");
                    telemetry.addLine("\nAVERAGE DELAY: (ms) " + averagePipe + "\n");
//                    telemetry.update();
                    //return a new Pose with the robotPose x,y,yaw. Here, the robotPose is the position of the TURRET relative to the field.
                    return new Pose2d(detection.robotPose.getPosition().x,detection.robotPose.getPosition().y,detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                }
            } else {
                //Handle apriltags which have null metadata. Should never encounter
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
//        telemetry.update();
        //this return statement is most likely redundant, but if a return did not occur in the loop, then return the first pose in currentDetections.
        if (!currentDetections.isEmpty())
        {
            AprilTagDetection detection = currentDetections.get(0);
            return new Pose2d(detection.robotPose.getPosition().x,detection.robotPose.getPosition().y,detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));

        }
        //return null if no apriltags are detected.
        return null;
    }
}   // end class