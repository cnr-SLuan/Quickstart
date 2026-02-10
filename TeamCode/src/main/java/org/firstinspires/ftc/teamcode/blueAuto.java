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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "blueAuto", group = "Autonomous")
@Configurable
public class blueAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Intake intake;
    private Shooter shooter;
    private Timer pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);
        intake.setGatePosition(0.0); // Preset Gate Closed
        setPathState(0);
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
                    shooter.shoot(); // 2. BOTH FLYWHEELS START
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
                    shooter.stop();
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