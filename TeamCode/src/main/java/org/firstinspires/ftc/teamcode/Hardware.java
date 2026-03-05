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

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;

public class Hardware {

    // ================= INTAKE =================
    public static DcMotorEx in1, in2;

    // ================= DRIVETRAIN =================

    public static DcMotorEx FL, FR, BL, BR;
    // ================= FLYWHEEL =================
    public static DcMotorEx bottom, top;


    // ================= TURRET =================


    public static CRServo leftTurret, rightTurret; //right isn't working

    // ================= HOOD+GATE =================

    public static Servo hood, gate;

    // ================= TURRET ENCODER =================

    AnalogInput analog_right;

    // ================= IMU =================

    IMU imu;

    // ================= INDICATOR LIGHT =================


    Servo hoodLight;

    // ================= PINPOINT =================
    public static SensorGoBildaPinpoint pinpoint;
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


        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorSimple.Direction.REVERSE);
        top = hardwareMap.get(DcMotorEx.class, "top");
        top.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);

        bottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        top.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        leftTurret = hardwareMap.get(CRServo.class, "leftTurret");
        rightTurret = hardwareMap.get(CRServo.class, "rightTurret");

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
