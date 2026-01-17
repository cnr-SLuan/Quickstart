package org.firstinspires.ftc.teamcode.aprilTagPrograms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "processorHolder")
public class processorHolder extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private LLResultTypes.BarcodeResult BarcodeResult;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
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
            telemetry.addData("Distance", llResult.getBotposeAvgDist());
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials.isEmpty()) {
                telemetry.addLine("No AprilTags");
            } else {
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    telemetry.addData("Distance", tag.getCameraPoseTargetSpace());//tells the space based on the april tag
                    telemetry.addData("Tag ID", tag.getFiducialId());
                    if (tag.getFiducialId() == 22){
                        telemetry.addData("Current Team", "Blue");
                    }
                    if (tag.getFiducialId() == 21){
                        telemetry.addData("Current Team", "Red");
                    }
                    else{
                        telemetry.addLine("Direct april tag not found.");
                    }
                    fiducials.clear();
                }
            }

        }
    }

}
