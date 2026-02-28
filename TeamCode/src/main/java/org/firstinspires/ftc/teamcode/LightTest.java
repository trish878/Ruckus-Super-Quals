package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RuckusTele.velocity;

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
    //AutoShooter autoShooter = new AutoShooter(hardwareMap);
    int tolerance = 20;
    public static double pos;
    @Override
    public void runOpMode() throws InterruptedException {
        hoodLight = hardwareMap.get(Servo.class, "hoodLight");
        //hoodLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); //Ryder you fucking idiot piece of shit
        waitForStart();
        while (opModeIsActive()) {
            hoodLight.setPosition(0.277);
            double Velocity = RuckusTele.bottom.getVelocity();
            if (30 > Velocity + tolerance) {
                hoodLight.setPosition(0.5);
                telemetry.addData("velocity",velocity);
            } else {
                hoodLight.setPosition(0.277);
            }
        }
    }
}
//MB TRIH