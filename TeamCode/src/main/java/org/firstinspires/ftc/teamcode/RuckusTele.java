package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "RuckusTele2")
@Config
public class RuckusTele extends LinearOpMode {
    public static DcMotorEx FL,FR, BR, BL, in1, in2, bottom,top;
    public static double D=10; //+0.1

    public static double P = 150; //+1

    public static double velocity;
    public static double intake, intake1;

    public static double minPower, gatepos;

    public static double power;

    CRServo right, left, f, zero, one, two;

    Servo gate, hood, hoodLight;



    double voltage;

    public static double hoodpos;
    double F = 32767.0 / 2340;
    int tolerance = 20;

    OverflowEncoder par0, par1, perp;

    AnalogInput analog_right,analog_left;
    public static double pow_r, pow_l;
    IMU  imu;

    double distance;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE
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
        hoodLight = hardwareMap.get(Servo.class, "hoodLight");


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");

        analog_right = hardwareMap.get(AnalogInput.class, "servopos");
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");




        in1 = hardwareMap.get(DcMotorEx.class, "in1");
        in1.setDirection(DcMotorEx.Direction.FORWARD);
        in2= hardwareMap.get(DcMotorEx.class, "in2");
        in2.setDirection(DcMotorEx.Direction.REVERSE);
        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorEx.Direction.REVERSE);

        top = hardwareMap.get(DcMotorEx.class, "top");
        top.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, D, F);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);



        //CURRENT AND PREVIOUS GAMEPAD OBJECTS
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad prevGamepad1 = new Gamepad();
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FR")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BL")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FL")));

        par0.setDirection(DcMotorSimple.Direction.REVERSE);//left is positive
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        AutoShooter autoShooter = new AutoShooter(hardwareMap);


        waitForStart();
        hoodLight.setPosition(0.3);


        imu.resetYaw();

        while (opModeIsActive()) {
            double Velocity = RuckusTele.bottom.getVelocity();
            if (velocity > Velocity + tolerance) {
                hoodLight.setPosition(0.3);
                telemetry.addData("velocity",velocity);
            } else {
                hoodLight.setPosition(0.5);
            }

            //imu.getRobotYawPitchRollAngles();
            /*telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch());
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll());
            telemetry.addData("par0", par0.getPositionAndVelocity().position);
            telemetry.addData("par1", par1.getPositionAndVelocity().position);
            telemetry.addData("perp", perp.getPositionAndVelocity().position);*/

            telemetry.addData("vbottom", bottom.getVelocity());
            telemetry.addData("top", top.getVelocity());

            /*two.setPower(pow_r);
            f.setPower(pow_l);
            voltage = analog_right.getVoltage();
            telemetry.addData("pos", ((voltage*360)/3.3));
            telemetry.update();


            top.setVelocity(velocity);
            bottom.setVelocity(velocity);

            telemetry.addData("vtop", top.getVelocity());
            telemetry.addData("vbottom", bottom.getVelocity());*/

            double tx = autoShooter.gettx();
            double ty = autoShooter.getty();
            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);

            voltage = analog_right.getVoltage();
            telemetry.addData("pos", voltage);



             f.setPower(power);
             two.setPower(power);





            telemetry.addData("power", power);
            //telemetry.addData("Tx", tx);
            telemetry.addData("power", power);


           telemetry.addData("tx", autoShooter.getty());




            distance = AutoShooter.getDistanceFromLimelightToGoal();


            if (autoShooter.isTracking()) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                telemetry.addData("Distance", AutoShooter.getDistanceFromLimelightToGoal());

            }
            telemetry.update();




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

            hood.setPosition(0.4);








            if(currentGamepad1.left_bumper){
                gate.setPosition(0.7);
                in1.setPower(1);
                in2.setPower(1);
            }
            else if(currentGamepad1.left_trigger>0.25){
                gate.setPosition(0.4);
                in1.setPower(1);
                in2.setPower(1);
            }else if(currentGamepad1.right_bumper){
                gate.setPosition(0.7);
                in1.setPower(-0.8);
                in2.setPower(-0.8);
                bottom.setVelocity(-1000);
                top.setVelocity(-1000);

            }else{
                gate.setPosition(0.7);
                in1.setPower(0);
                in2.setPower(0);

            }

            if(currentGamepad1.right_trigger>0.25){
                bottom.setVelocity(1700);
                top.setVelocity(1700);
            }else{
                bottom.setVelocity(0);
                top.setVelocity(0);
            }


            //if()




            //in1.setPower(0);
            //in2.setPower(0);




        }
    }


}
