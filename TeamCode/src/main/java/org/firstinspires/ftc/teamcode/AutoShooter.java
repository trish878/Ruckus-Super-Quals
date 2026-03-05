package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class AutoShooter {
    HardwareMap hardwareMap;
    CRServo f,two;
    public static DcMotorEx top, bottom;
    int alliance =5;
    private static Limelight3A limelight;

    public static IMU imu;

    static double limelightMountAngleDegrees  = 18;
    static double limelightLensHeightInches = 13.5;
    static double goalHeightInches = 29.5;
    static double angleToGoalDegrees;
    static double angleToGoalRadians;

    static double DistanceFromLimeLighttoGoalInches;

    public AutoShooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        f = hardwareMap.get(CRServo.class, "leftTurret");
        two = hardwareMap.get(CRServo.class, "rightTurret");
        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        bottom.setDirection(DcMotorEx.Direction.REVERSE);
        top = hardwareMap.get(DcMotorEx.class, "top");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        // 1. Define the directions based on your mounting
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

// 2. Create the orientation object
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

// 3. Initialize the IMU with these parameters
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public double gettx() {

        LLResult result = limelight.getLatestResult();
        double lastResult = result.getTx();
        if (result != null) {
            if (result.isValid()) {
                return result.getTx();
            }
        }
        return Double.NaN;
    }

    public double getta() {

        LLResult result = limelight.getLatestResult();
        double lastResult = result.getTa();
        if (result != null) {
            if (result.isValid()) {
                return result.getTa();
            }
        }
        return Double.NaN;
    }

    public void getxyz(){
        LLResult result = limelight.getLatestResult();
        if(result!=null){
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for(LLResultTypes.FiducialResult fr:fiducialResults){
                double x = fr.getRobotPoseFieldSpace().getPosition().x;
                double y = fr.getRobotPoseFieldSpace().getPosition().y;
                double z = fr.getRobotPoseFieldSpace().getPosition().z;

                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("z",z);
                telemetry.update();

            }
        }

    }

    public void getxy(){
        // First, tell Limelight which way your robot is facing
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

    }
    public double getty() {

        LLResult result = limelight.getLatestResult();
        double lastResult = result.getTy();
        if (result != null) {
            if (result.isValid()) {
                return result.getTy();
            }
        }
        return Double.NaN;
    }



    public boolean isTracking() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return true;
            }
        }
        return false;
    }


    public static double getDistanceFromLimelightToGoal(){
        LLResult result =limelight.getLatestResult();
        if(result != null && result.isValid()) {

            return result.getBotposeAvgDist();
        } else {
            // No tag found, return -1 or another error value
            return -1;
        }
    }




    public double gettxRed() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == 4) return result.getTx();
                }
            }
        }
        return Double.NaN;
    }







}
