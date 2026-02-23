package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Odom Turret Tracking")
@Config
public class OdomTurretTracking extends LinearOpMode {
    public static double kP = 1.5;
    public static double kD = 0.05;
    public static double kF = 0.3;      // angular velocity feedforward
    public static double kVision = 1;
    double lastError = 0.0;




    public static double TAG_X = 72;    //change this should work from anywhere not just
    public static double TAG_Y = 36;    //change this

    public static double red = 24;
    public static double blue = 20;

    static AnalogInput analog_right;

    public static double target;

    double error =5;

    //
    public static double ticksPerRev = 2.72; // change this to your turret motor + gearbox
    private ElapsedTime runtime = new ElapsedTime();

    CRServo two, f;

    double count = 0;


    public boolean track = true;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //AutoShooter autoShooter = new AutoShooter(hardwareMap);
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        double prev_voltage = 0;
        double voltage = 0;

        waitForStart();

        double lastTurretAngle = 0.0;
        double lastTime = getRuntime();

        while (opModeIsActive()) {

            /*prev_voltage = analog_right.getVoltage();
            telemetry.addData("voltage", prev_voltage);
            telemetry.addData("voltage");
            two.setPower(power);
            f.setPower(power);*/
        }
    }


}
