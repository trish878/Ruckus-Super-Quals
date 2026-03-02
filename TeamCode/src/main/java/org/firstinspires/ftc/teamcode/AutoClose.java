package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "FAR AUTO")
public class AutoClose extends LinearOpMode {

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    /* ===================== TUNABLES (MATCH TELEOP NAMES/BEHAVIOR) ===================== */

    // TeleOp uses hood.setPosition(0.4) every loop
    public static double hoodpos = 0.4;
    static ElapsedTime timer = new ElapsedTime();

    // TeleOp shooter velocity when RT held is 1700. You asked "shooter always on for entire auto"
    public static double velocity = AutoShooter.getDistanceFromLimelightToGoal()*292+1483;

    // TeleOp intake motors in1/in2 run at +1 when intaking/feeding
    public static double intakePower = 1.0;

    // TeleOp: gate 0.7 in "default/closed-ish", gate 0.4 when feeding (LT)
    public static double gateClosedPos = 0.7;
    public static double gateOpenPos = 0.40;

    // Gate pulse timing per shot (seconds)
    public static double gatePulseSec = 0.35;

    // Small settle before gate pulse (seconds)
    public static double settleBeforeGate = 0.15;

    // TeleOp PID values (same variable names)
    public static double D = 10;   // +0.1
    public static double P = 150;  // +1

    // TeleOp uses F = 32767/2340
    public static double F = 32767.0 / 2340.0;

    /* ===================== SHOOTER/INTAKE HARDWARE (MATCH TELEOP MAP NAMES) ===================== */
    public static class ShooterIO {
        private final DcMotorEx bottom, top, in1, in2;
        private final Servo hood, gate;

        public ShooterIO(HardwareMap hardwareMap) {
            // MATCH TELEOP HARDWARE NAMES
            in1 = hardwareMap.get(DcMotorEx.class, "in1");
            in1.setDirection(DcMotorEx.Direction.FORWARD);

            in2 = hardwareMap.get(DcMotorEx.class, "in2");
            in2.setDirection(DcMotorEx.Direction.REVERSE);

            bottom = hardwareMap.get(DcMotorEx.class, "bottom");
            bottom.setDirection(DcMotorEx.Direction.REVERSE);

            top = hardwareMap.get(DcMotorEx.class, "top");
            top.setDirection(DcMotorEx.Direction.FORWARD);

            gate = hardwareMap.get(Servo.class, "gate");
            hood = hardwareMap.get(Servo.class, "hood");

            // MATCH TELEOP PIDF SETUP
            PIDFCoefficients pidf = new PIDFCoefficients(P, 0, D, F);
            bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }


        // Shooter ON entire auto, gate closed, intake OFF
        public class Init implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    hood.setPosition(hoodpos);
                    gate.setPosition(gateClosedPos);

                    // shooter always spinning
                    bottom.setVelocity(velocity);
                    top.setVelocity(velocity);

                    // intake off by default
                    in1.setPower(0);
                    in2.setPower(0);

                    done = true;
                }
                return false;
            }
        }
        public Action init() { return new Init(); }

        // Intake ON (for intake paths)
        public class IntakeOn implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    in1.setPower(intakePower);
                    in2.setPower(intakePower);
                    done = true;
                }
                return false;
            }
        }
        public Action intakeOn() { return new IntakeOn(); }

        public class IntakeOff implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    in1.setPower(0);
                    in2.setPower(0);
                    done = true;
                }
                return false;
            }
        }
        public Action intakeOff() { return new IntakeOff(); }

        // Gate pulse shot:
        // - turn intake ON to feed
        // - open gate for gatePulseSec
        // - close gate, intake OFF

        public class PoseReset implements Action {
            private final Pose2d pose;
            private final MecanumDrive drive;
            private boolean done = false;

            PoseReset(MecanumDrive drive, Pose2d pose) {
                this.drive = drive;
                this.pose = pose;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    //drive.setPoseEstimate(pose);
                    done = true;
                }
                return false;
            }
        }
        public class ShootGatePulse implements Action {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;

                    // feed + open gate (teleop-style)
                    in1.setPower(intakePower);
                    in2.setPower(intakePower);
                    gate.setPosition(gateOpenPos);

                    timer.reset();
                }

                if (timer.seconds() >= gatePulseSec) {
                    gate.setPosition(gateClosedPos);
                    in1.setPower(0);
                    in2.setPower(0);
                    return false;
                }

                return true;
            }
        }
        public Action shootGatePulse() { return new ShootGatePulse(); }

        public class GateClose implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    gate.setPosition(gateClosedPos);
                    done = true;
                }
                return false;
            }
        }
        public Action gateClose() { return new GateClose(); }

        public class StopAll implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    gate.setPosition(gateClosedPos);
                    in1.setPower(0);
                    in2.setPower(0);
                    bottom.setVelocity(0);
                    top.setVelocity(0);
                    done = true;
                }
                return false;
            }
        }
        public Action stopAll() { return new StopAll(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // IMU init (TeleOp does this too; safe to keep for consistent drive behavior)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        ShooterIO shooter = new ShooterIO(hardwareMap);

        // Start pose EXACTLY matching your MeepMeep Close 15 Auto (kept from your code)
        Pose2d initialPose = new Pose2d(62, -8, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();
        if (isStopRequested()) return;

        //.setTangent(0)



        /* ===================== PATHS (UNCHANGED FROM YOUR AUTO) ===================== */
        TrajectoryActionBuilder shootPose = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(56,-8), Math.toRadians(0));
                //.setTangent(90)


//        TrajectoryActionBuilder intakeMiddle = shootPose.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(90)), Math.toRadians(90))
//                .lineToY(52);
//
//        TrajectoryActionBuilder outtakeMiddle = intakeMiddle.endTrajectory().fresh()
//                .lineToY(35)
//                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));
//
//        TrajectoryActionBuilder intakeGate1 = outtakeMiddle.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(11, 57, Math.toRadians(115)), Math.toRadians(90));
//
//        TrajectoryActionBuilder outtakeGate1 = intakeGate1.endTrajectory().fresh()
//                .lineToY(35)
//                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));
//
//        TrajectoryActionBuilder intakeGate2 = outtakeGate1.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(11, 57, Math.toRadians(115)), Math.toRadians(90));
//
//        TrajectoryActionBuilder outtakeGate2 = intakeGate2.endTrajectory().fresh()
//                .lineToY(35)
//                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));
//
//        TrajectoryActionBuilder intakeLeft = outtakeGate2.endTrajectory().fresh()
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(-11, 35, Math.toRadians(90)), Math.toRadians(90))
//                .lineToY(52);
//
//        TrajectoryActionBuilder outtakeLeft = intakeLeft.endTrajectory().fresh()
//                .lineToY(35)
//                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(210));
//
//        TrajectoryActionBuilder park = outtakeLeft.endTrajectory().fresh()
//                .setTangent(45)
//                .splineToSplineHeading(new Pose2d(-5, 30, Math.toRadians(45)), Math.toRadians(0));

        /* ===================== RUN ===================== */
        Actions.runBlocking(
                new SequentialAction(
                        // Shooter always ON, hood set, gate closed, intake OFF
                           shooter.init(),

                        // First shot pose (intake off), then shoot
                        shootPose.build()


                        //   new SleepAction(settleBeforeGate),
                        // shooter.shootGatePulse(),

                        // Intake middle (intake on during the path)
                        //new ParallelAction(intakeMiddle.build(), shooter.intakeOn()),
                        //  shooter.intakeOff(),

                        // Back + shoot
                        //outtakeMiddle.build(),//,
                        //  new SleepAction(settleBeforeGate),
                        //  shooter.shootGatePulse()//,

                        // Intake gate 1
                        //new ParallelAction(intakeGate1.build(), shooter.intakeOn()),
                        //shooter.intakeOff(),

                        // Back + shoot
                        //outtakeGate1.build(),
                        // new SleepAction(settleBeforeGate),
                        //shooter.shootGatePulse(),

                        // Intake gate 2
                        //new ParallelAction(intakeGate2.build(), shooter.intakeOn()),
                        //shooter.intakeOff(),

                        // Back + shoot
                        //  outtakeGate2.build(),
                        //  new SleepAction(settleBeforeGate),
                        //  shooter.shootGatePulse(),

                        // Intake left
                        //new ParallelAction(intakeLeft.build(), shooter.intakeOn()),
                        //  shooter.intakeOff(),

                        // Back + shoot
                        //outtakeLeft.build(),
                        // new SleepAction(settleBeforeGate),
                        //  shooter.shootGatePulse(),

                        // Park
                        //park.build()
                        //   shooter.gateClose()

                        // If you want everything OFF at the end:
                        // , shooter.stopAll()
                )
        );

        while (opModeIsActive()) { }
    }
}
