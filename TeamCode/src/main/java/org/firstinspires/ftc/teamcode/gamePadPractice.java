package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class gamePadPractice extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        //runs 50* a second
        double speedForward = -gamepad1.left_stick_y/2.0;
        double xDifference = gamepad1.left_stick_x - gamepad1.right_stick_x;
        double rearTriggerSum = gamepad1.left_trigger + gamepad1.right_trigger;

        telemetry.addData("left x", gamepad1.left_stick_x);
        telemetry.addData("left  y", speedForward);
        telemetry.addData("a button", gamepad1.a);

        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", gamepad1.right_stick_y);
        telemetry.addData("b button", gamepad1.b);
        telemetry.addData("X difference", xDifference);
        telemetry.addData("Rear Trigger Sum", rearTriggerSum);
    }
}

/*
1. add telemetry for the right joystick
2. add telemetry for the B button
3. add telemetry data to report the DIFFERENCE between x left joystick and x right joystick
4. add telemetry data to report the SUM between both rear triggers
 */