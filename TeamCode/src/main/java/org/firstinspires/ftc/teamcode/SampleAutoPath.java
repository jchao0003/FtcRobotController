package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SampleAutoPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT . ATTEMPT TO SCORE THE ARTIFACT

        START_TO_SHOOT,

        SHOOT_PRELOAD,

        DRIVE_TO_PRESET,

        PICKUP_PRESET,

        PRESET_TO_SHOOT
    }

    PathState pathState;

    private final Pose startPose = new Pose(22, 120, Math.toRadians(135));
    private final Pose nearShootPose = new Pose(35.5, 107, Math.toRadians(135));
    private final Pose nearPresetStart = new Pose(45.5, 81, Math.toRadians(0));
    private final Pose nearPresetEnd = new Pose(24.5, 81, Math.toRadians(0));


    private PathChain startToShoot, nearShootToNearPresetStart, nearPresetStartToNearPresetEnd, nearPresetEndToShoot;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, nearShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), nearShootPose.getHeading())
                .build();
        nearShootToNearPresetStart = follower.pathBuilder()
                .addPath(new BezierLine(nearShootPose, nearPresetStart))
                .setLinearHeadingInterpolation(nearShootPose.getHeading(), nearPresetStart.getHeading())
                .build();
        nearPresetStartToNearPresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(nearPresetStart, nearPresetEnd))
                .setLinearHeadingInterpolation(nearPresetStart.getHeading(), nearPresetEnd.getHeading())
                .build();
        nearPresetEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(nearPresetStart, nearShootPose))
                .setLinearHeadingInterpolation(nearPresetEnd.getHeading(), nearShootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case START_TO_SHOOT:
                follower.followPath(startToShoot, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer and make new state
                break;
            case SHOOT_PRELOAD:
                //check if follower is done with path
                //and check that 5 seconds has elapsed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(nearShootToNearPresetStart);
                    setPathState(PathState.DRIVE_TO_PRESET);
                }
                break;
            case DRIVE_TO_PRESET:
                //all done!
                if (!follower.isBusy()){
                    telemetry.addLine("To preload");
                    follower.followPath(nearPresetStartToNearPresetEnd);
                    setPathState(PathState.PICKUP_PRESET);
                }
                break;
            case PICKUP_PRESET:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up preload");
                    follower.followPath(nearPresetEndToShoot);
                    setPathState(PathState.PRESET_TO_SHOOT);
                }
                break;
            case PRESET_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("To launch, done");
                }
            default:
                telemetry.addLine("NO State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

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

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());

    }
}
