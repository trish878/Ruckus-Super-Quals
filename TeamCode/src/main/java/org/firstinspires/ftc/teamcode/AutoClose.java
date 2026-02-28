package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Auto")
@Config
public class Auto extends LinearOpMode {

    public enum Alliance { RED, BLUE }
    public Alliance alliance = Alliance.RED;

    @Override
    public void runOpMode() {

        int side = 1; // +1 RED, -1 BLUE

        // Alliance selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = Alliance.RED;
                side = 1;
            }
            if (gamepad1.right_bumper) {
                alliance = Alliance.BLUE;
                side = -1;
            }
            telemetry.addData("Alliance", alliance);
            telemetry.update();
        }

        Pose2d initialPose = new Pose2d(
                64,
                side * 13,
                side * Math.toRadians(180)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        /* =======================
           FIRST ROW INTAKE
           ======================= */
        TrajectoryActionBuilder intake1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-20, side*20, side*Math.toRadians(0)), side*Math.toRadians(1));
        //Intake Middle
        TrajectoryActionBuilder outtake1 = intake1.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, side*33, side*Math.toRadians(90)), side*Math.toRadians(90))
                .lineToY(side*52);

        /* =======================
           SECOND ROW INTAKE
           ======================= */
        TrajectoryActionBuilder intake2 = outtake1.endTrajectory().fresh()
                .lineToY(side*35)
                .splineToSplineHeading(new Pose2d(-20, side*20, side*Math.toRadians(0)), side*Math.toRadians(200));

        TrajectoryActionBuilder outtake2 = intake2.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, side*40, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(11, side*57, side*Math.toRadians(115)),side*Math.toRadians(90));

        /* =======================
           THIRD ROW INTAKE
           ======================= */
        TrajectoryActionBuilder intake3 = outtake2.endTrajectory().fresh()
                .lineToY(side*35)
                .splineToSplineHeading(new Pose2d(-20, side*20, side*Math.toRadians(0)), side*Math.toRadians(200));

        TrajectoryActionBuilder outtake3 = intake3.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, side*40, side*Math.toRadians(90)), side*Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(11, side*57, side*Math.toRadians(115)), side*Math.toRadians(90));


        TrajectoryActionBuilder intake4 = outtake3.endTrajectory().fresh()
                .lineToY(side*35)
                .splineToSplineHeading(new Pose2d(-20, side*20, side*Math.toRadians(0)), side*Math.toRadians(200));

        TrajectoryActionBuilder outtake4 = intake4.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, side*35, side*Math.toRadians(90)), side*Math.toRadians(90))
                .lineToY(side*52);

        TrajectoryActionBuilder intake5 = outtake4.endTrajectory().fresh()
                .lineToY(side*35)
                .splineToSplineHeading(new Pose2d(-20, side*20, side*Math.toRadians(90)), side*Math.toRadians(200));

        TrajectoryActionBuilder outtake5 = intake5.endTrajectory().fresh()
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-5, side*30, side*Math.toRadians(90)), side*Math.toRadians(0));



        /* =======================
           RUN ALL PATHS
           ======================= */
        Actions.runBlocking(
                new SequentialAction(
                        intake1.build(),
                        outtake1.build(),
                        intake2.build(),
                        outtake2.build(),
                        intake3.build(),
                        outtake3.build(),
                        intake4.build(),
                        outtake4.build(),
                        intake5.build(),
                        outtake5.build()
                )
        );
    }
}
