package org.firstinspires.ftc.teamcode.aprilTagPrograms;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutonomousNo1")
public class AutonomousNo1 extends OpMode{

    private DcMotor RL;
    private DcMotor RR;
    private DcMotor FL;
    private DcMotor FR;

    int maxDrivePower;
    float horizontalInput;
    float verticalInput;
    private Limelight3A limelight;
    private IMU imu;

    private static ElapsedTime myTimer = new ElapsedTime();

    //@Override
    public void init() {
        autoInit();
        start();
        loop();


    }

    //@Override
    private void autoInit() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        //set the pipeline index to 8
        limelight.pipelineSwitch(8); //pipeline index setting
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }

    }

    private void robotReverse() {
        FL.setPower(-1.0);
        RL.setPower(-1.0);
        FR.setPower(-1.0);
        RR.setPower(-1.0);
    }
    private void robotForward() {
        FL.setPower(1.0);
        RL.setPower(1.0);
        FR.setPower(1.0);
        RR.setPower(1.0);
    }

    private void robotRight() {
        FL.setPower(1.0);
        RL.setPower(1.0);
        FR.setPower(-1.0);
        RR.setPower(-1.0);
    }

    private void robotLeft() {
        FL.setPower(-1.0);
        RL.setPower(-1.0);
        FR.setPower(1.0);
        RR.setPower(1.0);
    }
    private void robotStop(){
        FL.setPower(0.0);
        RL.setPower(0.0);
        FR.setPower(0.0);
        RR.setPower(0.0);
    }
    private void autoRun() {
        robotReverse();
        stop();
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("April Tag: ", llResult.getPipelineIndex());
            if (llResult.getPipelineIndex() == 24){

            }
        }
    }
    private void teamRed(){
        robotLeft();
        while (myTimer.time() < 0.02){
            telemetry.addData("Time", "%.2f", myTimer.time());
            telemetry.update();
            if (myTimer.time() == 0.02){
                break;
            }
        }
        robotReverse();
        while (myTimer.time() < 0.15) {
            telemetry.addData("Time", "%.2f", myTimer.time());
            telemetry.update();
            if (myTimer.time() == 0.15) {
                break;
            }
        }
        robotStop();
    }
    //@Override
    private void gamepadInit() {
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        maxDrivePower = 1;
        RL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepadDrive();
    }
    private void processDriveInputs() {
        // -------------------Process Drive Inputs----------------
        FL.setPower(verticalInput * maxDrivePower + horizontalInput * maxDrivePower);
        RL.setPower(verticalInput * maxDrivePower + horizontalInput * maxDrivePower);
        FR.setPower(verticalInput * maxDrivePower - horizontalInput * maxDrivePower);
        RR.setPower(verticalInput * maxDrivePower - horizontalInput * maxDrivePower);
    }

    private void gamepadDrive() {
    // ---------------------Game Pad Drive-----------------
        horizontalInput = -gamepad1.left_stick_x;
        verticalInput = gamepad1.right_stick_y;
        processDriveInputs();
        telemetry.update();

    }
}
