package org.firstinspires.ftc.teamcode.aprilTagPrograms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/*
* The code initializes the limelight as a new object in the lime light class
* It starts the limelight and returns the target offsets in the telemetry while it has valid results
 */

@Autonomous
public class LimeLightTest extends OpMode {
    private Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");//initializes a new limelight variable
        limelight3A.pipelineSwitch(0); // 0 is blue and 1 is red
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult(); //LLResult is the what the limelight received
        if (llResult != null && llResult.isValid()){
            telemetry.addData("Target X offset", llResult.getTx());//Tx stands for Target x
            telemetry.addData("Target Y offset", llResult.getTy());//Ty stands for Target Y
            telemetry.addData("Target Area offset", llResult.getTa());//Ta stands for Target area
        }
    }
}
