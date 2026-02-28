//kD 0.0008
//kF 0.012
//kI = 0
// kP = 0.006
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import java.util.List;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



@TeleOp(name = "Angle Simple")
@Config
public class OdomRewrite extends LinearOpMode {

    /* ================= CONFIG ================= */

    Servo hood;


    public static double V_CCW_MAX =2.74;
    private static Limelight3A limelight;
    public static IMU imu;

    public static double kI;
    OverflowEncoder par0, par1, perp;

    Servo gate;

    boolean wrap=false;
    public static double V_WRAP_LOW = 0.64;
    static final double V_MAX = 3.304;
    static final double FULL_ROT_DEG = 360.0;
    public static double kF=0.012;

    public static double kD=0.0008;

    double integral =0;

    public static double kP=0.006;
    double odomTargetAngle = 0;

    double derror=0;
    double prev_error=0;





    double scaled_voltage;

    public static double hoodpos;
    public static double velocity;

    CRServo two, f;



    AnalogInput analog_right;

    public static double add_val, add_val2;
    DcMotorEx BL, BR, FL, FR;





    // ===== Turret angle tracking state =====
    static final double WRAP_THRESHOLD = 1.5; // volts

    double turretAngleDeg = 0.0;   // CONTINUOUS angle
    double prevVoltage = 0.0;
    boolean turretInitialized = false;

    public double delta_max =0;

    public static double full_rotation;

    DcMotorEx bottom, top;


    //2.76 volts
// true angle where wrap occurs

    public static double TAG_X = 72;    //change this should work from anywhere not just
    public static double TAG_Y = 36;

    double rotation = 2.76;
    double prev_voltage = 0;

    double D=10; //+0.1

    double P = 150; //+1

    double I = 0;
    double F = 32767.0 / 2340;

    DcMotorEx in1, in2;











    @Override
    public void runOpMode() throws InterruptedException {


        double voltage = 0;
        double count = 0;
        double cross = 0;
        AutoShooter autoShooter = new AutoShooter(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //AutoShooter autoShooter = new AutoShooter(hardwareMap);
        two = hardwareMap.get(CRServo.class, "two");
        in1 = hardwareMap.get(DcMotorEx.class, "in1");
        in2 = hardwareMap.get(DcMotorEx.class, "in2");
        in2.setDirection(DcMotorSimple.Direction.REVERSE);

        f = hardwareMap.get(CRServo.class, "f");
        hood = hardwareMap.get(Servo.class, "hood");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        gate = hardwareMap.get(Servo.class, "gate");

        limelight.pipelineSwitch(0);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BL")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FL")));

        par0.setDirection(DcMotorSimple.Direction.REVERSE);//left is positive
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        // 1. Define the directions based on your mounting
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

// 2. Create the orientation object
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

// 3. Initialize the IMU with these parameters
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        double volts = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();

        FL = hardwareMap.get(DcMotorEx.class, "FL"); //CONTROL HUB
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR = hardwareMap.get(DcMotorEx.class, "FR");//CONTROL HUB
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL = hardwareMap.get(DcMotorEx.class, "BL"); //CONTROL HUB
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR = hardwareMap.get(DcMotorEx.class, "BR"); //CONTROL HUB
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
        top = hardwareMap.get(DcMotorEx.class, "top");

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);

        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);





        waitForStart();

        while (opModeIsActive()) {
            prevGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            double x = currentGamepad1.left_stick_x;
            double y = -currentGamepad1.left_stick_y;
            double r = 0.8 * currentGamepad1.right_stick_x;

            double speedGain = 2;
            x = Math.signum(x) * (Math.cosh(Math.abs(x)) - 1);
            y = Math.signum(y) * (Math.cosh(Math.abs(y)) - 1);
            x *= speedGain;
            y *= speedGain;
            x = Math.max(-1, Math.min(1, x));
            y = Math.max(-1, Math.min(1, y));

            FL.setPower(y + x + r);
            FR.setPower(y - x - r);
            BL.setPower(y - x + r);
            BR.setPower(y + x - r);

            hood.setPosition(hoodpos);

            double v = analog_right.getVoltage();
            double angle = voltageToTurretAngleDeg(v);

            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double posx = pose.position.x;
            double posy = pose.position.y;

            telemetry.addData("x", posx);
            telemetry.addData("y", posy);

            telemetry.addData("heading", pose.heading);
            telemetry.addData("topv", top.getVelocity());
            telemetry.addData("bottomv", bottom.getVelocity());

            if(!wrap){
                double dx = TAG_X - pose.position.x;
                double dy = TAG_Y - pose.position.y;
                double angle2 = Math.atan2(dx, dy);
                odomTargetAngle = (angle2 - pose.heading.toDouble())*(180/PI);
                if(odomTargetAngle>180){
                    wrap = true;
                    odomTargetAngle = -1*(360-odomTargetAngle);
                }else if(odomTargetAngle<-180){
                    odomTargetAngle = 360-odomTargetAngle;
                    wrap=true;
                }
            }

            double error = angle-odomTargetAngle;
            if(wrap && Math.abs(error)<5){
                wrap =false;
            }
            telemetry.addData("targetAngle", odomTargetAngle);
            derror = error-prev_error;
            double power = kF * vel.angVel + kP*error + kD*derror;
            power = Math.max(-1, Math.min(1, power));

            f.setPower(power);
            two.setPower(power);
            prev_error = error;

            if (gamepad1.left_bumper) {
                gate.setPosition(0.7);
                in1.setPower(1);
                in2.setPower(1);
            } else if (gamepad1.left_trigger > 0.25) {
                gate.setPosition(0.4);
                in1.setPower(1);
                in2.setPower(1);
            } else if (gamepad1.right_bumper) {
                gate.setPosition(0.7);
                in1.setPower(-0.8);
                in2.setPower(-0.8);
                bottom.setVelocity(-1000);
                top.setVelocity(-1000);
            } else {
                gate.setPosition(0.7);
                in1.setPower(0);
                in2.setPower(0);
            }

            if(gamepad1.right_trigger>0.25){
                top.setVelocity(velocity);
                bottom.setVelocity(velocity);

            }else{
                top.setVelocity(0);
                bottom.setVelocity(0);
            }
            telemetry.addData("voltage", v);
            telemetry.addData("angle", angle);
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
        if (dv < -WRAP_THRESHOLD) {
            dv += V_MAX;
        }
        // CW wrap: 0 -> 3.3
        else if (dv > WRAP_THRESHOLD) {
            dv -= V_MAX;
        }

        double dAngle = (dv / V_CCW_MAX) * FULL_ROT_DEG;
        turretAngleDeg += dAngle;

        prevVoltage = v;
        return turretAngleDeg;
    }


}