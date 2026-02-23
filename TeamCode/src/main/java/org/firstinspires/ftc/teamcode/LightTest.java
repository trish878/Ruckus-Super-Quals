package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RuckusTele.velocity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Light Test")
@Config
public class LightTest extends LinearOpMode {
    RevBlinkinLedDriver hoodLight;
    AutoShooter autoShooter = new AutoShooter(hardwareMap);
    int tolerance = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            hoodLight = hardwareMap.get(RevBlinkinLedDriver.class, "hoodLight");
            hoodLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            double targetVelocity = AutoShooter.bottom.getVelocity();
            if (targetVelocity > velocity - tolerance) {//this assumes that the measurement of velocity stays the same from before you started doing everything
                hoodLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);//In addition, we must tune tolerance.
            } else {
                hoodLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

        }
    }
}
//MB TRIH