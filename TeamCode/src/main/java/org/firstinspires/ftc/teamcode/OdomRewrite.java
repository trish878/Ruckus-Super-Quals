package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RuckusTele.hoodpos;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.PoseVelocity2d;



@TeleOp(name = "Angle Simple")
@Config
public class OdomRewrite extends LinearOpMode {

    /* ================= CONFIG ================= */

    Servo hood;


    public static double V_CCW_MAX =2.74;
    public static double V_WRAP_LOW = 0.64;
    static final double V_MAX = 3.304;
    static final double FULL_ROT_DEG = 360.0;
    public static double kF;

    public static double kD;

    public static double kP;

    double derror=0;
    double prev_error=0;



    double scaled_voltage;

    CRServo two, f;



    AnalogInput analog_right;

    public static double add_val, add_val2;




    // ===== Turret angle tracking state =====
    static final double WRAP_THRESHOLD = 1.5; // volts

    double turretAngleDeg = 0.0;   // CONTINUOUS angle
    double prevVoltage = 0.0;
    boolean turretInitialized = false;

    public double delta_max =0;

    public static double full_rotation;


    //2.76 volts
// true angle where wrap occurs

    public static double TAG_X = 72;    //change this should work from anywhere not just
    public static double TAG_Y = 36;

    double rotation = 2.76;
    double prev_voltage = 0;




    //




    @Override
    public void runOpMode() throws InterruptedException {


        double voltage = 0;
        double count = 0;
        double cross = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //AutoShooter autoShooter = new AutoShooter(hardwareMap);
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        hood = hardwareMap.get(Servo.class, "hood");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        double volts = 0;


        waitForStart();

        // turret physically facing forward here
        //axon 1.217







        while (opModeIsActive()) {

            //ccw 0-2.8
            //cw 3.3-0.63

            double v = analog_right.getVoltage();
            double angle = voltageToTurretAngleDeg(v);

            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double dx = TAG_X - pose.position.x;
            double dy = TAG_Y - pose.position.y;
            double angle2 = Math.atan2(dx, dy);


            double odomTargetAngle = (angle2 - pose.heading.toDouble())*(180/PI);

            telemetry.addData("targetAngle", odomTargetAngle);
            double error = wrapDeg(angle-odomTargetAngle);
            derror = error-prev_error;


            double power = kF * vel.angVel + kP*error + kD*derror;

            power = Math.max(-1, Math.min(1, power));

            f.setPower(power);
            two.setPower(power);
            double prev_error = error;










            //telemetry.addData("target", odomTargetAngle);



            telemetry.addData("voltage", v);
            telemetry.addData("angle", angle);
            telemetry.update();






            //telemetry.addData("voltage", voltage);
            //telemetry.addData("rawvoltage", new_voltage);

            //

            telemetry.update();

        }
    }

    public double voltageToTurretAngleDeg(double v) {

        if (!turretInitialized) {
            prevVoltage = v;
            turretInitialized = true;
            return turretAngleDeg;
        }

        double dv = v - prevVoltage;

        // Detect wrap
        // CCW wrap: 3.3 -> 0
        if (dv < -WRAP_THRESHOLD) {
            dv += V_MAX;
        }
        // CW wrap: 0 -> 3.3
        else if (dv > WRAP_THRESHOLD) {
            dv -= V_MAX;
        }

        // Convert voltage delta to angle delta
        double dAngle = (dv / V_CCW_MAX) * FULL_ROT_DEG;
        turretAngleDeg += dAngle;

        prevVoltage = v;
        return turretAngleDeg;
    }

    static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

}

