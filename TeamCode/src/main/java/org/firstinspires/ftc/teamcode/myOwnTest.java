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

    private CRServo SR1, SR2;
    private DcMotorEx INTAKE, LN;
    private DcMotor RL, RR, FL, FR;

    double servoPower = 0.0;

    @Override
    public void runOpMode() {

        // --- DRIVE MOTORS ---
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);

        /*
        EDIT NO.1: set the right wheels to forward explicitly so the directions are corrected
         */
        FR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- LAUNCHER MOTOR ---
        LN = hardwareMap.get(DcMotorEx.class, "LN");
        LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LN.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- INTAKE MOTOR ---
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- CR SERVOS ---
        SR1 = hardwareMap.get(CRServo.class, "SR1");
        SR2 = hardwareMap.get(CRServo.class, "SR2");

        SR1.setDirection(CRServo.Direction.FORWARD);
        SR2.setDirection(CRServo.Direction.FORWARD);

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

            FL.setPower(fl / max);
            FR.setPower(fr / max);
            RL.setPower(rl / max);
            RR.setPower(rr / max);

            // --- INTAKE ---
            double rpm = (6000.0 / 360.0) * 60.0;
            if (gamepad1.a) {
                INTAKE.setVelocity(rpm, AngleUnit.DEGREES);
            } else {
                INTAKE.setPower(0);
            }

            // --- LAUNCHER ---
            if (gamepad1.x) {
                LN.setPower(0.7);
            } else {
                LN.setPower(0);
            }

            // --- CR SERVOS (manual control) ---
            double forwardPower = 1.0;  // move forward
            double stopPower = 0.0;     // neutral / stop

            // --- CR SERVOS (manual control) ---
            if (gamepad1.left_bumper) {
                // Forward
                servoPower = 1.0;
                SR1.setPower(servoPower);
                SR2.setPower(servoPower);
            }
            else if (gamepad1.right_bumper) {
                // Reverse
                servoPower = -1.0;
                SR1.setPower(servoPower);
                SR2.setPower(servoPower);
            }
            else {
                // No bumper → stop
                SR1.setPower(0.0);
                SR2.setPower(0.0);
            }
            // Apply power


            // --- Telemetry ---
            telemetry.addData("SR1 Power", SR1.getPower());
            telemetry.addData("SR2 Power", SR2.getPower());
            telemetry.update();
        }
    }
}
