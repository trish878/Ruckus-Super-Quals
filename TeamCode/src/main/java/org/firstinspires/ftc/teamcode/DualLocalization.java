package org.firstinspires.ftc.teamcode;
import static com.sun.tools.doclint.Entity.theta;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;


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
@TeleOp
@Config
public class DualLocalization extends LinearOpMode {

    public double ypos, xpos;
    public double aprilxblue =-56;
    public double aprilyblue=-56.6;

    public double aprilxred=-56;
    public double aprilyred=56;

    int alliance = 1; //1 is blue




    public double projx, projy;

    public double truex, truey;

    private Limelight3A limelight;
    private IMU imu;

    public double theta;
    public void runOpMode() {


        // Insert whatever initialization your own code does

        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Hardware hardware = new Hardware(hardwareMap);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                        )
                )
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);     // poll up to 250Hz max
        limelight.pipelineSwitch(0);
        limelight.start();






        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        waitForStart();

        while(opModeIsActive()) {


            // Make sure to call myLocalizer.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();


            double dx = pose.position.x;
            double dy = pose.position.y;
            double heading = pose.heading.toDouble();

            telemetry.addData("x", dx);
            telemetry.addData("y", dy);
            telemetry.addData("heading", heading);



            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = 0.8 * gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(y, -x),
                    -r
            ));


            hardware.FL.setPower(y + x + r);
            hardware.FR.setPower(y - x - r);
            hardware.BL.setPower(y - x + r);
            hardware.BR.setPower(y + x - r);

            if (imu != null) {
                // NOTE: You may need to adjust which yaw you use depending on your IMU orientation.
                double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                limelight.updateRobotOrientation(yawDeg);
                LLResult result =limelight.getLatestResult();

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addData("\n# AprilTags Detected", fiducials.size());

                for (LLResultTypes.FiducialResult tag : fiducials) {
                    int id = tag.getFiducialId();

                    telemetry.addLine(String.format("\n==== (ID %d) 36h11", id));

                    // Robot pose in FIELD space from THIS tag alone

                    // Robot pose relative to the TAG coordinate system (often useful for aligning)
                    Pose3D robotPoseTag = tag.getRobotPoseTargetSpace();
                    if (robotPoseTag != null) {
                        telemetry.addLine(String.format("TAG XYZ (m)    %6.3f %6.3f %6.3f",
                                robotPoseTag.getPosition().x,
                                robotPoseTag.getPosition().y,
                                robotPoseTag.getPosition().z));

                        xpos = robotPoseTag.getPosition().x;
                        ypos = robotPoseTag.getPosition().z*-1;

                    }
                }

            }

            theta = Math.atan2(ypos,xpos);
            projx = ypos*Math.cos(theta)*39.37; //adds to x position (goes right)
            projy = ypos*Math.sin(theta)*39.37;; //subtracts from y position (goes down)

            telemetry.addData("x", projx);
            telemetry.addData("y", projy);

            if(alliance == 1){
                //
                double xa1=aprilxblue+projy;
                double ya1 = aprilyblue+29+projx;
                telemetry.addData("x_localization", xa1);
                telemetry.addData("y_localization", ya1);
                telemetry.update();
            }






            //truex = aprilx+projx;
            //truey = aprily-projy;


            // Insert whatever teleop code you're using
        }
    }
}