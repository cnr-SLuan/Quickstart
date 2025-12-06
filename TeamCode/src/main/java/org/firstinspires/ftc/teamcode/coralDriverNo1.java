package org.firstinspires.ftc.teamcode;

//import to drive robot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//import to run flywheel
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "coralDriverNo1 (Blocks to Java)")
public class coralDriverNo1 extends LinearOpMode {
    private DcMotorEx LN;
    private DcMotor RL;
    private DcMotor RR;
    private DcMotor FL;
    private DcMotor FR;

    int maxDrivePower;
    float horizontalInput;
    float verticalInput;

    /**
     * Initializes the program; sets the max drive power to 1, sets
     * the left motors to reverse, calls the gamepad drive function
     */
    @Override
    public void runOpMode() {
        launcherPrep();
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        waitForStart();
        maxDrivePower = 1;
        FL.setDirection(DcMotor.Direction.REVERSE);

        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepadDrive();
    }
    

    private void launcherPrep() {
        // Motor hardwareMapname : LN)
        LN = hardwareMap.get(DcMotorEx.class, "LN");

        // Encoderless
        LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status:", "Ready 67 ");
        LN.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
    }
    /**
     * Sets the speed power for the motors when vertical and/or horizontal input is received
     */
    private void processDriveInputs() {
        // -------------------Process Drive Inputs----------------
        FL.setPower(verticalInput * maxDrivePower + horizontalInput * maxDrivePower);
        RL.setPower(verticalInput * maxDrivePower + horizontalInput * maxDrivePower);
        FR.setPower(verticalInput * maxDrivePower - horizontalInput * maxDrivePower);
        RR.setPower(verticalInput * maxDrivePower - horizontalInput * maxDrivePower);
    }

    /**
     * Initializes horizontal movement to be called with the left stick
     * moving along the x axis, and vertical movement to be called with the right
     * stick along the y axis
     */
    private void gamepadDrive() {
        // ---------------------Game Pad Drive-------------------
        while (opModeIsActive()) {
            horizontalInput = gamepad1.left_stick_x;
            verticalInput = -gamepad1.right_stick_y;
            processDriveInputs();
            telemetry.update();
        }
    }
}