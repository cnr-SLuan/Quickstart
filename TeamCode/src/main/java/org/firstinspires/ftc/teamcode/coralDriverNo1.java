    package org.firstinspires.ftc.teamcode;

    //import to drive robot
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;

    //import to run flywheel
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotor;

    //import to run servos
    //import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    @TeleOp(name = "coralDriverNo1 (Blocks to Java)")
    public class coralDriverNo1 extends LinearOpMode {
        //   private CRServo LS; //left servo
        //   private CRServo RS; //right servo
        private DcMotorEx INTAKE; //intake
        private DcMotorEx LN; //launcher
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
        //----------------RUNS OPMODE---------------
        @Override
        public void runOpMode() {
            launcherPrep();
            //servoPrep();
            intakePrep();
            telemetry.update();
            RL = hardwareMap.get(DcMotor.class, "RL");
            RR = hardwareMap.get(DcMotor.class, "RR");
            FL = hardwareMap.get(DcMotor.class, "FL");
            FR = hardwareMap.get(DcMotor.class, "FR");

            waitForStart();
            maxDrivePower = 1;
            FL.setDirection(DcMotor.Direction.REVERSE);
            RL.setDirection(DcMotor.Direction.REVERSE);

            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gamepadDrive();
            pilot();
        }

        //Calls all functions
        private void myLoop() {
            while (opModeIsActive()) {
                gamepadDrive();
                if (gamepad1.a) {
                    intakeArtifacts();
                }
                if (gamepad1.x) {
                    launchArtifact();
                }
            }
        }

        //-----------------------SERVOS-------------------------
        /*private void servoPrep() {
            LS = hardwareMap.get(CRServo.class, "LS");
            RS = hardwareMap.get(CRServo.class, "RS");
            LS.setPower(0.0);
            LS.setDirection(CRServo.Direction.REVERSE);
            RS.setPower(0.0);
            telemetry.addData("Servos Status", "Ready");
        }*/
        //-------------------FLYWHEEL-----------------------------
        private void launcherPrep() {
            // Motor hardwareMapname : LN)
            LN = hardwareMap.get(DcMotorEx.class, "LN");

            // Encoderless
            LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Launcher Status:", "Ready");
            LN.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //---------------------INTAKE--------------------------------
        private void intakePrep() {
            // Motor hardwareMapname : INT)
            INTAKE = hardwareMap.get(DcMotorEx.class, "INT");
            // Encoderless
            INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Intake Status:", "Ready");
            INTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        /**
         * Sets the speed power for the motors when vertical and/or horizontal input is received
         */
        //----------------------DRIVER----------------------------
        private void processDriveInputs() {
            // -------------------Process Drive Inputs----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            double fl = y + x + r;
            double fr = y - x - r;
            double rl = y - x + r;
            double rr = y + x - r;

            FL.setPower(fl);
            FR.setPower(fr);
            RL.setPower(rl);
            RR.setPower(rr);
        }

        /**
         * Initializes horizontal movement to be called with the left stick
         * moving along the x axis, and vertical movement to be called with the right
         * stick along the y axis
         */
        private void gamepadDrive() {
            // ---------------------Game Pad Drive-------------------
            horizontalInput = gamepad1.right_stick_x;
            verticalInput = gamepad1.left_stick_y;
            processDriveInputs();
            telemetry.update();

        }

        //-----------------------LAUNCH FUNCTION-----------------------
        private void launchArtifact() {
            double degrees = 6000.0;
            double rpm = (degrees / 360) * 60;//makes 1000 rpm
            // X keypad
            if (gamepad1.x) {
                //LN.setPower(1); // %100 speed
                LN.setVelocity(rpm, AngleUnit.DEGREES);
                //LS.setPower(1.0);
                //RS.setPower(1.0);
                telemetry.addData("Status", "Running (Full Speed)");
            } else {
                LN.setPower(0.0);
                //LS.setPower(0.0);
                //RS.setPower(0.0);
                telemetry.addData("Status", "Stopped");
            }
            telemetry.update();
        }

        //----------------------INTAKE FUNCTION-------------------------
        private void intakeArtifacts() {
            double degrees = 6000.0;
            double rpm = (degrees / 360) * 60;//makes 1000 rpm
            if (gamepad1.a) {
                INTAKE.setVelocity(rpm, AngleUnit.DEGREES);
                telemetry.addData("Status", "Running (Full Speed)");
            } else {
                INTAKE.setPower(0.0);
                telemetry.addData("Status", "Stopped");
            }
        }

        private void pilot() {
            while (opModeIsActive()) {
                horizontalInput = gamepad1.left_stick_x;
                verticalInput = -gamepad1.right_stick_y;
                processDriveInputs();
                telemetry.update();

                double degrees = 6000.0;
                double rpm = (degrees / 360) * 60;//makes 1000 rpm
                if (gamepad1.a) {
                    INTAKE.setVelocity(rpm, AngleUnit.DEGREES);
                    telemetry.addData("Status", "Running (Full Speed)");
                }
                if (gamepad1.x) {
                    //LN.setPower(1); // %100 speed
                    LN.setVelocity(rpm, AngleUnit.DEGREES);
                    //LS.setPower(1.0);
                    //RS.setPower(1.0);
                    telemetry.addData("Status", "Running (Full Speed)");
                } else {
                    INTAKE.setPower(0.0);

                    LN.setPower(0.0);
                    //LS.setPower(0.0);
                    // RS.setPower(0.0);
                    telemetry.addData("Status", "Stopped");
                }
            }
        }
    }