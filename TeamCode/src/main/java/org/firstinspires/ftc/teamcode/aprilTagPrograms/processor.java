package org.firstinspires.ftc.teamcode.aprilTagPrograms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "processor")
public class processor extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    LLResult llResult;

    double min;
    double max;
    int temp;

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
        llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
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
    private void redTeam(){
        ArrayList<ArrayList<Double>> myStats = new ArrayList<>();
        ArrayList<Double> miniStats = new ArrayList<Double>();
        if (gamepad1.dpadUpWasPressed()){
            miniStats.add(llResult.getTx());
            miniStats.add(llResult.getTy());
            miniStats.add(llResult.getTa());
            myStats.add(miniStats);
            while (!miniStats.isEmpty()){
                Collections.sort(miniStats);
                min = miniStats.get(0);
                temp = miniStats.size();
                temp =- 1;
                max = miniStats.get(temp);
            }
        }
        else{
            telemetry.addLine("Nothing for now.");
        }

    }

}
