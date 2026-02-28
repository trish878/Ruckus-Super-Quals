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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "SetPosTurret")
@Config
public class SetPositionOdomTurret extends LinearOpMode {

    /* ================= CONFIG ================= */

    Servo hood;

    Servo t1, t2;


    public static double V_CCW_MAX =2.74;
    private static Limelight3A limelight;
    public static IMU imu;

    public static double kI;

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


    //2.76 volts
// true angle where wrap occurs

    public static double TAG_X = 72;    //change this should work from anywhere not just
    public static double TAG_Y = 36;

    double rotation = 2.76;
    double prev_voltage = 0;

    double full = 0.8;




    //






    @Override
    public void runOpMode() throws InterruptedException {


        double voltage = 0;
        double count = 0;
        double cross = 0;
        AutoShooter autoShooter = new AutoShooter(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //AutoShooter autoShooter = new AutoShooter(hardwareMap);
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        hood = hardwareMap.get(Servo.class, "hood");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
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



        waitForStart();

        while (opModeIsActive()) {


            /*autoShooter.getxy();
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
            limelight.updateRobotOrientation(robotYaw);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }
            }*/


            /*LLResult result = limelight.getLatestResult();
            if(result!=null){
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for(LLResultTypes.FiducialResult fr:fiducialResults) {
                    double x = fr.getRobotPoseTargetSpace().getPosition().x;
                    double y = fr.getRobotPoseTargetSpace().getPosition().y;
                    double z = fr.getRobotPoseTargetSpace().getPosition().z;

                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.addData("z", z);
                    telemetry.update();

                }
            }*/
            //ccw 0-2.8
            //cw 3.3-0.63


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

            double t1Position = t1.getPosition();
            double t2Position = t2.getPosition();
            double v = (t1Position+t2Position)/2;

            double angle = servoPosToAngle(v);



            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double posx = pose.position.x;
            double posy = pose.position.y;

            telemetry.addData("x", posx);
            telemetry.addData("y", posy);




            /*double heading = pose.heading.toDouble();

            telemetry.addData("posx", "posy", )*/


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

            //combine odom distance with limelight distance?
            telemetry.addData("targetAngle", odomTargetAngle);

            derror = error-prev_error;


            double power = kF * vel.angVel + kP*error + kD*derror;

            power = Math.max(-1, Math.min(1, power));

            f.setPower(power);
            two.setPower(power);
            prev_error = error;










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

    public double servoPosToAngle(double pos) {
        double half = full/2;
        double sign =0;
        if (pos<half){
            sign = 1;
        }else if (pos>half){
            sign = -1;
        }
        double angle = (half-pos)*360/full; //CONVERT TO ANGLE
        return angle;
    }


}