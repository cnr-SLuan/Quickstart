package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

@TeleOp(name = "collabAttempt")
public class collabAttempt extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private CRServo SR1;   // Continuous rotation servo
    private Servo SR2;     // Positional servo (0..180 deg)
    double sr2Pos = 0.5;                      // start at ~90°
    final double SR2_STEP = 75.0 / 180.0;     // ≈ 0.4167

    // --- MOTORS ---
    private DcMotorEx INTAKE;
    private DcMotorEx LN, LN2;
    private DcMotor RL, RR, FL, FR;
    private Timer pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // --- INTAKE MOTOR ---
        INTAKE = hardwareMap.get(DcMotorEx.class, "INT");
        INTAKE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        INTAKE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INTAKE.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- LAUNCHER MOTOR ---
        LN = hardwareMap.get(DcMotorEx.class, "LN");
        LN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LN.setDirection(DcMotorSimple.Direction.FORWARD);

        LN2 = hardwareMap.get(DcMotorEx.class, "LN2");
        LN2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LN2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LN2.setDirection(DcMotorSimple.Direction.FORWARD);

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);
        SR2.setGatePosition(0.0); // Preset Gate Closed
        setPathState(0);

        SR2 = hardwareMap.get(Servo.class, "SR2");
        SR2.setPosition(sr2Pos);   // start at ~90°
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2: // DRIVE FINISHED
                if (!follower.isBusy()) {
                    intake.intake(); // 1. INTAKE MOTOR FIRST
                    setPathState(3);
                }
                break;
            case 3: // START SHOOTERS
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (gamepad1.x) {
                        LN.setPower(0.7);
                        LN2.setPower(0.7);
                    } else {
                        LN.setPower(0);
                        LN2.setPower(0);
                    } // 2. BOTH FLYWHEELS START
                    setPathState(4);
                }
                break;
            case 4: // OPEN GATE
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.setGatePosition(0.5); // 3. GATE OPENS TO 90 DEG
                    setPathState(5);
                }
                break;
            case 5: // SCORING WINDOW
                if (pathTimer.getElapsedTimeSeconds() > 5.0) {
                    intake.stop();
                    LN.setPower(0.0);
                    intake.setGatePosition(0.0);
                    setPathState(-1);
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Path1, Path2;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 8, Math.toRadians(90)), new Pose(56, 87, Math.toRadians(135))))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 87, Math.toRadians(135)), new Pose(30, 113, Math.toRadians(135)))).build();
        }
    }
}

