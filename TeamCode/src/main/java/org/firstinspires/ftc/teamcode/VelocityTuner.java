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

@TeleOp(name = "LiftTest")
@Config
public class VelocityTuner extends LinearOpMode {
    DcMotorEx  bottom,top, in1, in2;

    Servo f, two;


    Servo rightservo, leftservo;
    CRServo lift1, lift2;

    public static double pos_right, pos_left;
    public static double D=10; //+0.1

    public static double P = 150; //+1

    public static double velocity;
    public static double gate_pos;

    public static double pos;



    public static double I;

    public static double hoodpos;
    double F = 32767.0 / 2340;

    Servo gate;

    public static double inpower;

    OverflowEncoder par0, par1, perp;

    IMU  imu;

    double distance;
    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException {

        in1 = hardwareMap.get(DcMotorEx.class, "in1");
        in1.setDirection(DcMotorEx.Direction.FORWARD);
        in2= hardwareMap.get(DcMotorEx.class, "in2");
        gate= hardwareMap.get(Servo.class, "gate");
        rightservo= hardwareMap.get(Servo.class, "rightservo");
        leftservo= hardwareMap.get(Servo.class, "leftservo");

        lift1 = hardwareMap.get(CRServo.class, "servo");
        lift2 = hardwareMap.get(CRServo.class, "crservo");
        f = hardwareMap.get(Servo.class, "f");
        two = hardwareMap.get(Servo.class, "two");


        in2.setDirection(DcMotorEx.Direction.REVERSE);

        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorEx.Direction.REVERSE);

        top = hardwareMap.get(DcMotorEx.class, "top");
        top.setDirection(DcMotorEx.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);


        waitForStart();


        while (opModeIsActive()) {
            f.setPosition(pos);
            two.setPosition(pos);
            rightservo.setPosition(pos_right);
            leftservo.setPosition(pos_left);

            bottom.setVelocity(velocity);
            top.setVelocity(velocity);
            in1.setPower(inpower);
            in2.setPower(inpower);
            gate.setPosition(gate_pos);
            double bv = bottom.getVelocity();
            double tv = top.getVelocity();
            telemetry.addData("velocity bottom", bv);
            telemetry.addData("velocity top", tv);
            telemetry.update();



        }
    }


}
