package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Lifter test")
@Config
public class LifterTest extends LinearOpMode {
//    Servo gate;
    public static double topv, bottomv, hoodv = 0;
    public static double position = 0.1;
    Servo hood;
    DcMotorEx top, bottom;
    AnalogInput servopos;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            top = hardwareMap.get(DcMotorEx.class, "top");
            bottom = hardwareMap.get(DcMotorEx.class, "bottom");
            hood = hardwareMap.get(Servo.class, "hood");
            top.setDirection(DcMotorSimple.Direction.REVERSE);
            top.setPower(topv);
            bottom.setPower(bottomv);
            hood.setPosition(hoodv);
            telemetry.addData("bottom", bottom.getVelocity());
            telemetry.addData("top", top.getVelocity());
            telemetry.update();


        }
    }
}
//MB TRIH