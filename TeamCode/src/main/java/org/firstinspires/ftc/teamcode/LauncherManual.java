package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LauncherManual")
public class LauncherManual extends OpMode {

    private DcMotorEx LN;

    @Override
    public void init() {
        // Motor hardwareMapname : LN)
        LN = hardwareMap.get(DcMotorEx.class, "LN");

        // Encoderless
        LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status:", "Ready 67 ");
        LN.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }

    @Override
    public void loop() {
        // X keypad
        double degrees = 6000.0;
        double rpm = (degrees/360)*60;//makes 1000 rpm
        if (gamepad1.x) {
            LN.setPower(1.0); // %100 speed
            LN.setVelocity(rpm, AngleUnit.DEGREES);

            telemetry.addData("Status", "Running (Full Speed)");
        } else {
            LN.setPower(0.0);
            telemetry.addData("Status", "Stopped");
        }

        telemetry.update();
    }
}



