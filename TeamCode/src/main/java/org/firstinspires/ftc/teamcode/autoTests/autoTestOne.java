package org.firstinspires.ftc.teamcode.autoTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class autoTestOne extends OpMode{
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState(){
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
                    telemetry.addLine("Path1 complete.");
                }
                break;
            default:
                telemetry.addLine("No State COmmanded");
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
    }

    @Override
    public void loop(){

    }
}
