package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    // ================= INTAKE =================
    DcMotorEx in1, in2;

    // ================= DRIVETRAIN =================

    DcMotorEx FL, FR, BL, BR;
    // ================= FLYWHEEL =================
    DcMotorEx bottom, top;

    // ================= TURRET =================


    CRServo f, two;

    // ================= HOOD+GATE =================

    Servo hood, gate;

    // ================= TURRET ENCODER =================

    AnalogInput analog_right;

    // ================= IMU =================

    IMU imu;

    // ================= INDICATOR LIGHT =================


    Servo hoodLight;

    // ================= VELOCITY PIDF VALUES =================


    double D=10; //+0.1

    double P = 150; //+1

    double I = 0;
    double F = 32767.0 / 2340;

    public Hardware(HardwareMap hardwareMap){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        in1 = hardwareMap.get(DcMotorEx.class, "in1");

        in2 = hardwareMap.get(DcMotorEx.class, "in2");
        in2.setDirection(DcMotorSimple.Direction.REVERSE);


        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
        top = hardwareMap.get(DcMotorEx.class, "top");

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);

        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        f = hardwareMap.get(CRServo.class, "f");
        two = hardwareMap.get(CRServo.class, "two");

        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");

        analog_right = hardwareMap.get(AnalogInput.class, "servopos");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        ));

        imu.resetYaw();



    }
}
