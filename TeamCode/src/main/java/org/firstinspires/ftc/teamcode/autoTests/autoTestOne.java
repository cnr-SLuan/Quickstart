package org.firstinspires.ftc.teamcode.autoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

//----------MECHANICS---------------
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class autoTestOne extends OpMode{
    //---------Mechanics Initialization-----------
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // --- SERVOS ---

    private Servo SR2;     // Positional servo (0..180 deg)

    // --- MOTORS ---
    private DcMotorEx INTAKE;
    private DcMotorEx LN, LN2;
    private DcMotor RL, RR, FL, FR;
    private DcMotor SR;
    // --- SR2 (+75 / -75 degree control) ---

    final double SR2_STEP = 75.0 / 180.0;     // ≈ 0.4167
    // --- TURN SLOWDOWN (rotation only) ---
    private static double TURN_SCALE = 0.5; // 0.25 slower, 0.5 medium, 1.0 original

    public enum PathState{
        //START POSITION/END POSITION
        //drive > Movement state
        //Shoot > Attempt to score artifact
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }

    PathState pathState;
    private final Pose startPose = new Pose(122.221142162819, 124.70473876063181);
    private final Pose shootPose = new Pose(85.82746051032808,89.50182260024302);
    private PathChain driveStartShootPos;

    public void buildPaths(){
        //coordinates for startpose and endingpose
        driveStartShootPos = follower.pathBuilder()
        .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()){
                    //TODO add logic to flywheel shooter
                    LN.setPower(0.7);
                    LN2.setPower(0.7);
                    telemetry.addLine("Path1 complete.");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanics
        double sr2Pos = 0.0;    // start at ~0°
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
        SR2 = hardwareMap.get(Servo.class, "SR2");
        SR = hardwareMap.get(DcMotor.class, "SR");

        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SR.setDirection(DcMotor.Direction.FORWARD);
        SR2.setPosition(sr2Pos);   // start at ~90°

        telemetry.addData("Status", "Ready");
        telemetry.update();


        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
