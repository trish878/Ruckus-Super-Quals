package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Light Test")
@Config
public class LightTest extends LinearOpMode {
    Servo hoodLight;
    AutoShooter autoShooter = new AutoShooter(hardwareMap);
    int tolerance = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            hoodLight = hardwareMap.get(Servo.class, "hoodLight");

        }
    }
}
//MB TRIH