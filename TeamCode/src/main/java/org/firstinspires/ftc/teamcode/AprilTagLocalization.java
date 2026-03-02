package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

@TeleOp(name = "Concept: Limelight AprilTag Localization", group = "Concept")
//@Disabled
public class AprilTagLocalization extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu; // optional (only needed for MegaTag2)

    // Replace these IDs with whatever you consider "Obelisk" tags for your use case.
    // (Limelight FiducialResult provides ID, not a metadata name.)
    private static final Set<Integer> OBELISK_TAG_IDS = new HashSet<Integer>() {{
        // add(tagId);
        // add(1); add(2);
    }};

    // Set this to the pipeline index on your Limelight that runs AprilTags + Full 3D (MegaTag).
    private static final int APRILTAG_PIPELINE_INDEX = 0;

    @Override
    public void runOpMode() {

        // Config name MUST match your Robot Configuration (e.g. "limelight")
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);     // poll up to 250Hz max
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        // Optional: if you have an IMU configured and want MegaTag2, uncomment and set the name.
        // imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("Limelight ready.");
        telemetry.addLine("dpad_up = resume polling, dpad_down = pause polling");
        telemetry.addLine("Touch START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Pause / resume polling (similar spirit to stop/resume streaming)
            if (gamepad1.dpad_down) {
                limelight.pause();
            } else if (gamepad1.dpad_up) {
                limelight.start(); // resumes polling
            }

            telemetryLimelight();

            telemetry.update();
            sleep(20);
        }

        // Clean shutdown
        limelight.stop();
        limelight.shutdown();
        limelight.close();
    }

    private void telemetryLimelight() {
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Polling", limelight.isRunning());
        telemetry.addData("ms since update", limelight.getTimeSinceLastUpdate());

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("Limelight: No valid result");
            return;
        }

        telemetry.addData("Pipeline", result.getPipelineIndex());
        telemetry.addData("Staleness (ms)", result.getStaleness());

        // ===== Overall robot pose from MegaTag1 (botpose) =====
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addLine("\n== MegaTag1 botpose (field) ==");
            telemetry.addData("XYZ (m)", "%.3f, %.3f, %.3f",
                    botpose.getPosition().x,
                    botpose.getPosition().y,
                    botpose.getPosition().z);
            telemetry.addData("YPR (deg)", "%.1f, %.1f, %.1f",
                    botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                    botpose.getOrientation().getPitch(AngleUnit.DEGREES),
                    botpose.getOrientation().getRoll(AngleUnit.DEGREES));
            telemetry.addData("Tags Used", result.getBotposeTagCount());
            telemetry.addData("AvgDist (m)", "%.2f", result.getBotposeAvgDist());
        } else {
            telemetry.addLine("\n== MegaTag1 botpose: null (enable Full 3D in pipeline) ==");
        }

        // ===== Optional: MegaTag2 (requires yaw fed from IMU) =====
        //see whats field centric vs distance from center
        //check if projy and projx detect the right x and y position from the center of limelight
        //if they do translate to roadrunner using constant apriltag position x and y and new pose 2d reset
        // set red and blue goals for apriltag position
        //drive train using localization practice
        //redo odometry with new localization technique //set red and blue goals
        //retune turret odom PID
        //test when odom is out of tune with target/ OR WHEN ITS ANGLE READS WRONG
        //see if you can calculate distance traveled after apriltag localization to get new distance
        //MAJOR ASSUMPTION ASSUME TURRET IS FACING APRILTAG AND ODOM TRACKING IS RIGHT TO RELOCALIZE DISTANCE
        //WE CAN SHOOT WITHOUT TRACKING BUT WITH ONLY VERY MINIMAL SLOW ROTATIONAL MOVEMENT
        //button to lock or at 0 or something
        if (imu != null) {
            // NOTE: You may need to adjust which yaw you use depending on your IMU orientation.
            double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(yawDeg);

            Pose3D botposeMt2 = result.getBotpose_MT2();
            if (botposeMt2 != null) {
                telemetry.addLine("\n== MegaTag2 botpose (field, IMU-fused) ==");
                telemetry.addData("XYZ (m)", "%.3f, %.3f, %.3f",
                        botposeMt2.getPosition().x,
                        botposeMt2.getPosition().y,
                        botposeMt2.getPosition().z);
                telemetry.addData("YPR (deg)", "%.1f, %.1f, %.1f",
                        botposeMt2.getOrientation().getYaw(AngleUnit.DEGREES),
                        botposeMt2.getOrientation().getPitch(AngleUnit.DEGREES),
                        botposeMt2.getOrientation().getRoll(AngleUnit.DEGREES));
            }
        }

        // ===== Per-tag detections (similar to your loop over AprilTagDetection) =====
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("\n# AprilTags Detected", fiducials.size());

        for (LLResultTypes.FiducialResult tag : fiducials) {
            int id = tag.getFiducialId();

            telemetry.addLine(String.format("\n==== (ID %d) 36h11", id));

            // Only use tags that are NOT "Obelisk" (by ID here)
            if (OBELISK_TAG_IDS.contains(id)) {
                telemetry.addLine("Ignored (Obelisk ID)");
                continue;
            }

            // Robot pose in FIELD space from THIS tag alone
            Pose3D robotPoseField = tag.getRobotPoseFieldSpace();
            if (robotPoseField != null) {
                telemetry.addLine(String.format("FIELD XYZ (m)  %6.3f %6.3f %6.3f",
                        robotPoseField.getPosition().x,
                        robotPoseField.getPosition().y,
                        robotPoseField.getPosition().z));
                telemetry.addLine(String.format("FIELD YPR (deg)%6.1f %6.1f %6.1f",
                        robotPoseField.getOrientation().getYaw(AngleUnit.DEGREES),
                        robotPoseField.getOrientation().getPitch(AngleUnit.DEGREES),
                        robotPoseField.getOrientation().getRoll(AngleUnit.DEGREES)));
            }

            // Tag center info (degrees/pixels)
            telemetry.addLine(String.format("tx/ty (deg)  %6.2f %6.2f",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees()));
            telemetry.addLine(String.format("center (px)  %6.1f %6.1f",
                    tag.getTargetXPixels(), tag.getTargetYPixels()));

            // Robot pose relative to the TAG coordinate system (often useful for aligning)
            Pose3D robotPoseTag = tag.getRobotPoseTargetSpace();
            if (robotPoseTag != null) {
                telemetry.addLine(String.format("TAG XYZ (m)    %6.3f %6.3f %6.3f",
                        robotPoseTag.getPosition().x,
                        robotPoseTag.getPosition().y,
                        robotPoseTag.getPosition().z));
                //y and z are swapped

            }
        }

        telemetry.addLine("\nkey:");
        telemetry.addLine("FIELD pose = robot pose in FTC field coordinates (meters)");
        telemetry.addLine("TAG pose   = robot pose relative to that AprilTag (meters)");
    }
}
//5.63 inches from center of limelight to center of turret
//1.6 inches center of robot to center of the shooter
