package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "blueGoalAuto (Blocks to Java)")
public class blueGoalAuto extends LinearOpMode {

    private DcMotor INTAKE;
    private Servo SR2;
    private DcMotor SR;
    private DcMotor LN;
    private DcMotor LN2;
    private DcMotor FL;
    private DcMotor RL;
    private DcMotor FR;
    private DcMotor RR;

    double sr2Pos;

    /**
     * Describe this function...
     */
    private void shoot() {
        sleep(2000);
        INTAKE.setPower(0.6);
        SR2.setPosition(sr2Pos);
        SR.setPower(1);
        sleep(5000);
        INTAKE.setPower(0);
        LN.setPower(0);
        LN2.setPower(0);
        SR.setPower(0);
        sleep(2000);
    }

    /**
     * Describe this function...
     */
    private void redGoal() {
        LN.setPower(0.58);
        LN2.setPower(0.58);
        reverse();
        sleep(1400);
        stop2();
        shoot();
        leaveWhiteLine();
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        int SR2_STEP;

        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        SR2 = hardwareMap.get(Servo.class, "SR2");
        SR = hardwareMap.get(DcMotor.class, "SR");
        LN = hardwareMap.get(DcMotor.class, "LN");
        LN2 = hardwareMap.get(DcMotor.class, "LN2");
        FL = hardwareMap.get(DcMotor.class, "FL");
        RL = hardwareMap.get(DcMotor.class, "RL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        RR = hardwareMap.get(DcMotor.class, "RR");

        sr2Pos = 0.5;
        SR2_STEP = 75 / 180;
        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SR.setDirection(DcMotor.Direction.FORWARD);
        LN2.setDirection(DcMotor.Direction.REVERSE);
        LN.setDirection(DcMotor.Direction.FORWARD);
        INTAKE.setDirection(DcMotor.Direction.FORWARD);
        INTAKE.setPower(0);
        FL.setDirection(DcMotor.Direction.REVERSE);
        RL.setDirection(DcMotor.Direction.REVERSE);
        // Put initialization blocks here.
        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LN2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            SR2.setPosition(sr2Pos);
            redGoal();
            telemetry.update();
            while (opModeIsActive()) {
                // Put loop blocks here.
            }
        }
    }

    /**
     * Describe this function...
     */
    private void forward() {
        FL.setPower(1);
        RL.setPower(1);
        FR.setPower(1);
        RR.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void right() {
        FL.setPower(1);
        RL.setPower(1);
        FR.setPower(-1);
        RR.setPower(-1);
    }

    /**
     * Describe this function...
     */
    private void leaveWhiteLine() {
        right();
        sleep(250);
        forward();
        sleep(350);
        stop2();
    }

    /**
     * Describe this function...
     */
    private void left() {
        FL.setPower(-1);
        RL.setPower(-1);
        FR.setPower(1);
        RR.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void reverse() {
        FL.setPower(-0.5);
        RL.setPower(-0.5);
        FR.setPower(-0.5);
        RR.setPower(-0.5);
    }

    /**
     * Describe this function...
     */
    private void stop2() {
        FL.setPower(0);
        RL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
    }
}