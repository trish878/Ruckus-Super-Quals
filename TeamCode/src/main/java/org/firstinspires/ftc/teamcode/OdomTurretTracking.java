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
        double prev_voltage =0;
        double voltage = 0;

        waitForStart();

        double lastTurretAngle = 0.0;
        double lastTime = getRuntime();

        while (opModeIsActive()) {
            if(count<5){
                prev_voltage = analog_right.getVoltage();
                count++;
            }

            double rawVoltage = analog_right.getVoltage();
            double new_voltage= rawVoltage;
            double delta_voltage = new_voltage - prev_voltage;
            if(delta_voltage>=3.302){
                delta_voltage = delta_voltage-3.302;
            }

            voltage+=delta_voltage;
            voltage%=2.72;
            prev_voltage = new_voltage;
            telemetry.addData("voltage", voltage);
            telemetry.update();

            /*double currentAngle = (turretVoltage/2.72)*360;
            double derror= currentAngle-target;


            double power = kP * derror;


            if(currentAngle>error){
                two.setPower(power);
                f.setPower(power);
            }else{
                two.setPower(0);
                f.setPower(0);
            }



            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double dx = TAG_X - pose.position.x; //need to make tag x and tag y
            double dy = TAG_Y - pose.position.y;
            double angle = Math.atan2(dy, dx); //is it dy/dx or dx/dy

            double odomTargetAngle = normalizeRadians(angle - pose.heading.toDouble());



            double targetAngle = normalizeRadians(odomTargetAngle);

            double turretAngle = (turretVoltage / 2.72) * 2.0 * Math.PI;
            double error = normalizeRadians(targetAngle - turretAngle);

            double dError = (error - lastError) / dt;
            lastError = error;

            double errorTolerance = Math.toRadians(5);
            double turretPower;
            if (Math.abs(error) < errorTolerance) {
                turretPower = 0.0;
            } else {
                turretPower = kP * error + kD * dError;
                turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
            }
            turretPower *= -1;

            two.setPower(turretPower);
            f.setPower(turretPower);
            //f.setPower(turretPower);*/

            /*telemetry.addData("Robot X (in)", pose.position.x);
            telemetry.addData("Robot Y (in)", pose.position.y);
            telemetry.addData("Robot Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngle)); //tune pid according to this
            telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle)); //tune pid according to this
            telemetry.addData("Turret Power", turretPower);
            telemetry.update();

            count++;
            if(runtime.seconds()>=1 && track){
                track = false;
                telemetry.addData("loops", count);
                telemetry.update();
            }*/
        }
    }

    //so rotation isn't weird
    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }



    //need to figure out the rotating anglen
}
