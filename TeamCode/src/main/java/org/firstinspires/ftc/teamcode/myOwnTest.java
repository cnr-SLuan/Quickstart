package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



/**
 * December 2025
 * Valeria's code by the way, do not mess with please.
 * this code is so I can mess with it and don't alter MyTest, which actually works.
 * NEVER USE THIS FOR COMPETITIONS.
 */

@TeleOp(name = "myOwnTest")
public class myOwnTest extends LinearOpMode {

    private CRServo intakeServoLeft, intakeServoRight;
    private DcMotorEx intakeMotor, flywheelMotor;
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    double servoPower = 0.0;


    @Override
    public void runOpMode() {
        // --- DRIVE MOTORS ---
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        /*
        EDIT NO.1: set the right wheels to forward explicitly so the directions are corrected
         */
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- LAUNCHER MOTOR ---
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- INTAKE MOTOR ---
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- CR SERVOS ---
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");

        intakeServoLeft.setDirection(CRServo.Direction.FORWARD);
        intakeServoRight.setDirection(CRServo.Direction.FORWARD);

        // Neutral position
        //SR1.setPower(0.5);
        //SR2.setPower(0.5);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- DRIVE CONTROLS ---
            //EDIT NO.4: turn is on left joystick and stafe is on right joystick
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.left_stick_x*0.2; //EDIT NO.2: x is meant to be strafing, and r is rotation
            double x = gamepad1.right_stick_x*1.1;

            /*
            EDIT NO.3: normalizes the power values by creating four new variables instead of
            initiating the motors, which sets the power to stay under 1 by finding the maximum power
            in all of the wheels and dividing the values by the max to set the power.
             */

            double fl = y + x + r;
            double fr = y - x - r;
            double rl = y - x + r;
            double rr = y + x - r;

            // Normalize so no value exceeds 1
            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(rl), Math.abs(rr)))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(rl / max);
            backRight.setPower(rr / max);

            // --- INTAKE ---
            double rpm = (6000.0 / 360.0) * 60.0;
            if (gamepad1.a) {
                intakeMotor.setVelocity(rpm, AngleUnit.DEGREES);
            } else {
                intakeMotor.setPower(0);
            }

            // --- LAUNCHER ---
            if (gamepad1.x) {
                flywheelMotor.setPower(0.7);
            } else {
                flywheelMotor.setPower(0);
            }

            // --- CR SERVOS (manual control) ---
            double forwardPower = 1.0;  // move forward
            double stopPower = 0.0;     // neutral / stop

            // --- CR SERVOS (manual control) ---
            if (gamepad1.left_bumper) {
                // Forward
                servoPower = 1.0;
                intakeServoLeft.setPower(servoPower);
                intakeServoRight.setPower(servoPower);
            }
            else if (gamepad1.right_bumper) {
                // Reverse
                servoPower = -1.0;
                intakeServoLeft.setPower(servoPower);
                intakeServoRight.setPower(servoPower);
            }
            else {
                // No bumper → stop
                intakeServoLeft.setPower(0.0);
                intakeServoRight.setPower(0.0);
            }
            // Apply power


            // --- Telemetry ---
            telemetry.addData("intakeServoLeft Power", intakeServoLeft.getPower());
            telemetry.addData("intakeServoRight Power", intakeServoRight.getPower());
            telemetry.update();
        }
    }
}
