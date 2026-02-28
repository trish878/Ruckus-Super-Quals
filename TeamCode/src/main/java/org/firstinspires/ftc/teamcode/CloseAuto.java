package org.firstinspires.ftc.teamcode.MAIN;

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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class CloseAuto extends LinearOpMode {

    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // ====== TUNABLES ======
    public static double hoodposition = 0.4;

    // shooter always ON during entire auto (ticks/sec)
    public static double shootVelClose = 2200;

    // intake only ON during intake paths + when shooting
    public static double intakePower = 1.0;

    // gate servo positions (setPosition)
    public static double gateOpenPos = 0.62;
    public static double gateClosedPos = 0.40;

    // how long to open gate per shot
    public static double gatePulseSec = 0.18;

    // small settle after getting into shoot pose
    public static double settleBeforeGate = 0.1;

    public class Shooter {
        private DcMotorEx outtake_top, outtake_bottom, intake;
        private Servo hood, gate;

        public Shooter(HardwareMap hardwareMap) {
            outtake_top = hardwareMap.get(DcMotorEx.class, "outtake_top");
            outtake_top.setDirection(DcMotorSimple.Direction.REVERSE);

            outtake_bottom = hardwareMap.get(DcMotorEx.class, "outtake_bottom");
            intake = hardwareMap.get(DcMotorEx.class, "intake");

            hood = hardwareMap.get(Servo.class, "hood");
            gate = hardwareMap.get(Servo.class, "gate");
        }

        // shooter velocity ON for entire auto, gate closed, intake OFF by default
        public class Init implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    hood.setPosition(hoodposition);
                    gate.setPosition(gateClosedPos);

                    outtake_bottom.setVelocity(shootVelClose);
                    outtake_top.setVelocity(shootVelClose);

                    intake.setPower(0);

                    done = true;
                }
                return false;
            }
        }
        public Action init() { return new Init(); }

        public class IntakeOn implements Action {
            private boolean done = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!done) {
                    intake.setPower(intakePower);
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
                    intake.setPower(0);
                    done = true;
                }
                return false;
            }
        }
        public Action intakeOff() { return new IntakeOff(); }

        // Shoots by:
        // 1) turning intake ON (feeds)
        // 2) opening gate for gatePulseSec
        // 3) closing gate
        // 4) turning intake OFF
        public class ShootGatePulse implements Action {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    intake.setPower(intakePower);
                    gate.setPosition(gateOpenPos);
                    timer.reset();
                }

                if (timer.seconds() >= gatePulseSec) {
                    gate.setPosition(gateClosedPos);
                    intake.setPower(0);
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
                    intake.setPower(0);
                    outtake_bottom.setVelocity(0);
                    outtake_top.setVelocity(0);
                    done = true;
                }
                return false;
            }
        }
        public Action stopAll() { return new StopAll(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);

        // NO TURRET CODE AT ALL

        // Start pose EXACTLY matching your MeepMeep Close 15 Auto
        Pose2d initialPose = new Pose2d(-61.5, 38, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();
        if (isStopRequested()) return;

        // ====== PATHS (MATCH YOUR MEEP MEEP CLOSE 15 AUTO EXACTLY) ======
        TrajectoryActionBuilder shootPose = drive.actionBuilder(initialPose)
                // First Shot
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(1));

        TrajectoryActionBuilder intakeMiddle = shootPose.endTrajectory().fresh()
                // Intake Middle
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(52);

        TrajectoryActionBuilder outtakeMiddle = intakeMiddle.endTrajectory().fresh()
                // Outtake Middle
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));

        TrajectoryActionBuilder intakeGate1 = outtakeMiddle.endTrajectory().fresh()
                // Intake Gate 1
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(11, 57, Math.toRadians(115)), Math.toRadians(90));

        TrajectoryActionBuilder outtakeGate1 = intakeGate1.endTrajectory().fresh()
                // Outtake Gate 1
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));

        TrajectoryActionBuilder intakeGate2 = outtakeGate1.endTrajectory().fresh()
                // Intake Gate 2
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, 40, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(11, 57, Math.toRadians(115)), Math.toRadians(90));

        TrajectoryActionBuilder outtakeGate2 = intakeGate2.endTrajectory().fresh()
                // Outtake Gate 2
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(200));

        TrajectoryActionBuilder intakeLeft = outtakeGate2.endTrajectory().fresh()
                // Intake Left
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(-11, 35, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(52);

        TrajectoryActionBuilder outtakeLeft = intakeLeft.endTrajectory().fresh()
                // Outtake Left
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(-20, 20, Math.toRadians(135)), Math.toRadians(210));

        TrajectoryActionBuilder park = outtakeLeft.endTrajectory().fresh()
                // Park
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-5, 30, Math.toRadians(45)), Math.toRadians(0));

        // ====== RUN ======
        // Pattern:
        // - shooter always spinning
        // - intake ON only during intake paths and during each shot
        // - gate opens only during shot pulse
        Actions.runBlocking(
                new SequentialAction(
                        shooter.init(),

                        // Drive to first shot pose (intake OFF), then shoot (intake ON only during pulse)
                        shootPose.build(),
                        new SleepAction(settleBeforeGate),
                        shooter.shootGatePulse(),

                        // Intake Middle (intake ON during path)
                        new ParallelAction(
                                intakeMiddle.build(),
                                shooter.intakeOn()
                        ),
                        shooter.intakeOff(),

                        // Back to shoot pose (intake OFF), then shoot
                        outtakeMiddle.build(),
                        new SleepAction(settleBeforeGate),
                        shooter.shootGatePulse(),

                        // Intake Gate 1
                        new ParallelAction(
                                intakeGate1.build(),
                                shooter.intakeOn()
                        ),
                        shooter.intakeOff(),

                        // Back + shoot
                        outtakeGate1.build(),
                        new SleepAction(settleBeforeGate),
                        shooter.shootGatePulse(),

                        // Intake Gate 2
                        new ParallelAction(
                                intakeGate2.build(),
                                shooter.intakeOn()
                        ),
                        shooter.intakeOff(),

                        // Back + shoot
                        outtakeGate2.build(),
                        new SleepAction(settleBeforeGate),
                        shooter.shootGatePulse(),

                        // Intake Left
                        new ParallelAction(
                                intakeLeft.build(),
                                shooter.intakeOn()
                        ),
                        shooter.intakeOff(),

                        // Back + shoot
                        outtakeLeft.build(),
                        new SleepAction(settleBeforeGate),
                        shooter.shootGatePulse(),

                        // Park (intake OFF)
                        park.build(),
                        shooter.gateClose()

                        // if you want everything OFF at end:
                        // , shooter.stopAll()
                )
        );

        while (opModeIsActive()) { }
    }
}
