package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MyTest")
public class myOwnTest extends LinearOpMode {

    // --- SERVOS ---
    private CRServo SR1;   // Continuous rotation servo
    private Servo SR2;     // Positional servo (0..180 deg)

    // --- MOTORS ---
    private DcMotorEx INTAKE;
    private DcMotorEx LN, LN2;
    private DcMotor RL, RR, FL, FR;

    double servoPower = 0.0;

    // --- SR2 (+75 / -75 degree control) ---
    double sr2Pos = 0.5;                      // start at ~90°
    final double SR2_STEP = 75.0 / 180.0;     // ≈ 0.4167
    boolean ltWasPressed = false;
    boolean rtWasPressed = false;
    boolean rt2WasPressed = false;

    @Override
    public void runOpMode() {

        // --- DRIVE MOTORS ---
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- LAUNCHER MOTORS ---
        LN = hardwareMap.get(DcMotorEx.class, "LN");
        LN2 = hardwareMap.get(DcMotorEx.class, "LN2");

        LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LN2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LN.setDirection(DcMotorSimple.Direction.FORWARD);
        LN2.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- INTAKE ---
        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- SERVOS ---
        SR1 = hardwareMap.get(CRServo.class, "SR1");
        SR2 = hardwareMap.get(Servo.class, "SR2");

        SR1.setDirection(CRServo.Direction.FORWARD);
        SR2.setPosition(sr2Pos);   // start at ~90° (aka open)

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- MECANUM DRIVE ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            FL.setPower(y + x + r);
            FR.setPower(y - x - r);
            RL.setPower(y - x + r);
            RR.setPower(y + x - r);

            // --- INTAKE ---
            //double rpm = (6000.0 / 360.0) * 60.0;
            if (gamepad1.a) {
                INTAKE.setPower(0.7);
            } else {
                INTAKE.setPower(0);
            }

            // --- LAUNCHER ---
            if (rtWasPressed) {
                LN.setPower(0.55);
                LN2.setPower(0.55);
            } else {
                LN.setPower(0);
                LN2.setPower(0);
            }
            //--------JAM FIX-------------
            //NOTE: This will ONLY change the direction.
            boolean rt2Pressed = gamepad2.right_trigger > 0.5;
            if (rt2Pressed){
                LN.setDirection(DcMotorSimple.Direction.REVERSE);
                LN2.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (!rt2Pressed){
                LN.setDirection(DcMotorSimple.Direction.FORWARD);
                LN2.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            rt2WasPressed = rt2Pressed;

            // --- SR1 (CR SERVO) ---
            if (gamepad1.x) {
                SR1.setPower(1.0);
            } else if (gamepad1.b) {
                SR1.setPower(-1.0);
            } else {
                SR1.setPower(0.0);
            }

            // --- SR2 (POSITION SERVO ±75°) ---
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            boolean ltPressed = gamepad2.left_trigger > 0.5;

            if (ltPressed) {
                sr2Pos = Math.min(1.0, sr2Pos + SR2_STEP);
            }

            if (gamepad2.left_bumper) {
                sr2Pos = Math.max(0.0, sr2Pos - SR2_STEP);
            }

            ltWasPressed = ltPressed;
            rtWasPressed = rtPressed;
            SR2.setPosition(sr2Pos);


            // --- TELEMETRY ---
            telemetry.addData("SR1 Power", SR1.getPower());
            telemetry.addData("SR2 Position", sr2Pos);
            telemetry.addData("SR2 Degrees", sr2Pos * 180.0);
            telemetry.update();
        }
    }
}
