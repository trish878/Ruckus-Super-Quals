package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RuckusTele.hoodpos;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Angle Simple")
@Config
public class OdomRewrite extends LinearOpMode {

    /* ================= CONFIG ================= */

    Servo hood;

    public static double hoodpos;

    CRServo two, f;

    AnalogInput analog_right;

    public static double add_val, add_val2;

    public static double power;

    public static double full_rotation;

    double TAG_X;
    double TAG_Y;

    //2.76 volts

    public static double V_WRAP_START = 2.85;  // volts at start of "top segment"
    public static double V_MAX        = 3.30;  // max volts before it wraps to 0
    public static double ANGLE_WRAP   = 56.0;  // true angle where wrap occurs

    double rotation = 2.76;
    double prev_voltage;




    //




    @Override
    public void runOpMode() throws InterruptedException {


        double voltage = 0;
        double count = 0;
        double cross = 0;








        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //AutoShooter autoShooter = new AutoShooter(hardwareMap);
        two = hardwareMap.get(CRServo.class, "two");
        f = hardwareMap.get(CRServo.class, "f");
        hood = hardwareMap.get(Servo.class, "hood");
        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        analog_right = hardwareMap.get(AnalogInput.class, "servopos");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        double volts = 0;



        waitForStart();

        // turret physically facing forward here
        //axon 1.217







        while (opModeIsActive()) {
            //max is 360




            double currentAngle;

            double new_voltage= analog_right.getVoltage();
            telemetry.addData("rawvoltage",new_voltage );

            double delta_voltage = new_voltage - prev_voltage;
            if(delta_voltage>=3.302){
                delta_voltage = delta_voltage-3.302;
            }




            //track how many times 3.3 is crossed
            //add other value

            voltage+=delta_voltage;
            voltage%=2.72;
            prev_voltage = new_voltage;

            /*double new_voltage= analog_right.getVoltage();

            if(new_voltage>=3.3){
                cross++;
                volts = cross*3.304;
            }else{
                volts=cross*3.304 + new_voltage;
            }

            volts%=2.72;*/




















            //telemetry.addData("angle", (v/2.82)*360);
            telemetry.addData("voltage", voltage);
            telemetry.addData("rawvoltage", new_voltage);





            //0 to 360




            ///2.84 volts is a full rotation


            f.setPower(power);
            two.setPower(power);







            hood.setPosition(hoodpos);

            //

            telemetry.update();

        }
    }

}

