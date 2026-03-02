package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Math.clamp;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Autonomous(name = "Straight Auto")
@Config
public class AutoFar extends LinearOpMode {
    Servo hood;
    boolean foundTag = false;

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
    double lastGoodDistance = Double.NaN;
    boolean haveLastGoodDistance = false;

    public static double kD=0.0008;

    double integral =0;

    public static double kP=-0.006;
    double odomTargetAngle = 0;

    double derror=0;
    double prev_error=0;





    double scaled_voltage;

    public static double hoodpos;
    double velocity;

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

    public enum Alliance {
        BLUE, RED
    }

    public OdomRewrite.Alliance alliance;


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
    double distance = 0;
    AutoShooter autoShooter = new AutoShooter(hardwareMap);
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        LLResult result;
        double tty = 0;
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
        CRServo two, f;
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        in1 = hardwareMap.get(DcMotorEx.class, "in1");
        in2 = hardwareMap.get(DcMotorEx.class, "in2");
        in2.setDirection(DcMotorSimple.Direction.REVERSE);
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        hood = hardwareMap.get(Servo.class, "hood");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
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

        double volts = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();

        waitForStart();


        timer.reset();
        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);
        if (timer.seconds() > 750) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = OdomRewrite.Alliance.BLUE;
                telemetry.addData("Alliance", "BLUE selected");
                telemetry.update();
            } else if (gamepad1.right_bumper) {
                alliance = OdomRewrite.Alliance.RED;
                telemetry.addData("Alliance", "RED selected");
                telemetry.update();
            }

        }
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            foundTag = false;

            if (result != null && result.isValid()) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();

                    if ((alliance == OdomRewrite.Alliance.BLUE && id == 20) || (alliance == OdomRewrite.Alliance.RED  && id == 24)) {

                        Pose3D botpose = result.getBotpose();
                        double dist = result.getBotposeAvgDist();

                        if (botpose != null && Double.isFinite(dist)) {
                            lastGoodDistance = dist;
                            haveLastGoodDistance = true;
                        }

                        tty = autoShooter.getty();
                        foundTag = true;
                        break; // stop after first matching tag
                    }
                }
            }

            if (!foundTag) {
                tty=0;
            }

            // ================= HOOD (SAFE, USE LAST GOOD) =================
            if (haveLastGoodDistance) {
                double hoodCmd = 0.539
                        - 0.233 * lastGoodDistance
                        + 0.0447 * lastGoodDistance * lastGoodDistance;

                if (Double.isFinite(hoodCmd)) {
                    hoodCmd = clamp(hoodCmd, 0.0, 0.5);
                    hood.setPosition(hoodCmd);
                }
            }

            // ================= AUTO SHOOTER POWER (SAFE) =================


            if (Double.isFinite(tty)) {
                double pwr = clamp(kP * tty, -1.0, 1.0);
                f.setPower(pwr);
                two.setPower(pwr);
            } else {
                f.setPower(0);
                two.setPower(0);
            }
            top.setVelocity(velocity);
            bottom.setVelocity(velocity);
            if(timer.milliseconds()>5000){
                in1.setPower(1);
                in2.setPower(1);
            }




        }
    }
}