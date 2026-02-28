package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "RuckusTele")

@Config
public class RuckusTele extends LinearOpMode {

    // ================= HARDWARE =================


    public static double velocity;
    public static double gate_pos;
    public static double hoodpos;

    OverflowEncoder par0, par1, perp;




    // ================= TURRET PID =================
    public static double kP_turret = -0.005;
    public static double kF_turret = -0.009;
    public static double kI_turret = 0.0;
    public static double kD_turret = -0.002;

    double turretIntegral = 0;
    double turretLastError = 0;

    ElapsedTime turretTimer = new ElapsedTime();

    // Deadband to prevent jitter
    public static double tyDeadband = 0.3;

    // ================= OTHER =================
    double voltage;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        AutoShooter autoShooter = new AutoShooter(hardwareMap);

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FL")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BR")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));

        //par0.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        turretTimer.reset();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {


            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();



            double dx = pose.position.x;
            double dy = pose.position.y;
            double heading = pose.heading.toDouble();

            telemetry.addData("x", dx);
            telemetry.addData("y", dy);
            telemetry.addData("heading", heading);
            telemetry.addData("topv", hardware.top.getVelocity());
            telemetry.addData("bottomv", hardware.bottom.getVelocity());

            telemetry.addData("par0", par0.getPositionAndVelocity().position);
            telemetry.addData("par1", par1.getPositionAndVelocity().position);
            telemetry.addData("perp", perp.getPositionAndVelocity().position);


            /*if (gamepad1.options) {
                drive.pose = new Pose2d(0, 0, 0); // Resets the "Ghost" robot to zero
            }*/





            // ================= DRIVE =================
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = 0.8 * gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(y, -x),
                    -r
            ));

// IMPORTANT: You must call update() to refresh the localizer


            hardware.FL.setPower(y + x + r);
            hardware.FR.setPower(y - x - r);
            hardware.BL.setPower(y - x + r);
            hardware.BR.setPower(y + x - r);

            // ================= HOOD =================
            hardware.hood.setPosition(hoodpos);

            // ================= LIMELIGHT =================
            double ty = autoShooter.getty();
            double tx = autoShooter.gettx();
            double ta = autoShooter.getta();
            double distance = AutoShooter.getDistanceFromLimelightToGoal();

            telemetry.addData("distance", distance);

            telemetry.addData("ty", ty);
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);

            //x
            //y
            //distance = Math.sqrt(x^2 + y^2)



            /*if(hardware.top.getVelocity()>velocity-20 && hardware.top.getVelocity()<velocity+20 && hardware.bottom.getVelocity()>velocity-20 && hardware.bottom.getVelocity()<velocity+20) {
                hardware.hoodLight.setPosition(0.5);
            }*/

            // ================= TURRET PID =================
            if (!Double.isNaN(ty)) {

                double error = ty;   // target = 0
                double dt = turretTimer.seconds();
                turretTimer.reset();

                if (Math.abs(error) < tyDeadband) {
                    turretIntegral = 0;
                    hardware.f.setPower(0);
                    hardware.two.setPower(0);
                } else {
                    turretIntegral += error * dt;
                    turretIntegral = Math.max(-1, Math.min(1, turretIntegral));

                    double derivative = (error - turretLastError) / dt;
                    turretLastError = error;

                    double output =
                            kP_turret * error +
                                    kI_turret * turretIntegral +
                                    kD_turret * derivative + kF_turret * vel.angVel;

                    output = Math.max(-1, Math.min(1, output));

                    hardware.f.setPower(output);
                    hardware.two.setPower(output);
                }

            } else {
                turretIntegral = 0;
                turretLastError = 0;
                hardware.f.setPower(0);
                hardware.two.setPower(0);
            }

            // ================= INTAKE / GATE =================
            if (gamepad1.left_bumper) {
                hardware.gate.setPosition(0.7);
                hardware.in1.setPower(1);
                hardware.in2.setPower(1);
            } else if (gamepad1.left_trigger > 0.25) {
                hardware.gate.setPosition(0.4);
                hardware.in1.setPower(1);
                hardware.in2.setPower(1);
            } else if (gamepad1.right_bumper) {
                hardware.gate.setPosition(0.7);
                hardware.in1.setPower(-0.8);
                hardware.in2.setPower(-0.8);
                hardware.bottom.setVelocity(-1000);
                hardware.top.setVelocity(-1000);
            } else {
                hardware.gate.setPosition(0.7);
                hardware.in1.setPower(0);
                hardware.in2.setPower(0);
            }

            if(gamepad1.right_trigger>0.25){
                hardware.top.setVelocity(velocity);
                hardware.bottom.setVelocity(velocity);

            }else{
                hardware.top.setVelocity(0);
                hardware.bottom.setVelocity(0);
            }
            voltage = hardware.analog_right.getVoltage();
            telemetry.addData("Turret Voltage", voltage);

            telemetry.update();
        }
    }
}