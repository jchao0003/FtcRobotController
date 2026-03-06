package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoRedFrontNoGateV2 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    int shotsToFire = 5;

    RobotHardwareV2 robotHardware;

    private double fudgeFactor = Math.toRadians(15);




    public void hardwareInit(){
        robotHardware = new RobotHardwareV2();
        RobotInitializerV2.initializeRobot(hardwareMap, robotHardware);

        robotHardware.resetMechanisms();
    }


    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT . ATTEMPT TO SCORE THE ARTIFACT

        START,
        START_TO_PRELOAD_SHOOT,
        LAUNCH_PRELOAD,
        WAIT_FOR_LAUNCH_PRELOAD,
        SHOOT_PRELOAD_TO_START_CLOSE,
        START_CLOSE_TO_END_CLOSE,
        END_CLOSE_TO_SHOOT_CLOSE,
        ADJUST_LAUNCH_CLOSE,
        LAUNCH_CLOSE,
        WAIT_FOR_LAUNCH_CLOSE,
        SHOOT_CLOSE_TO_START_MID,
        START_MID_TO_END_MID,
        END_MID_TO_SHOOT_MID,
        ADJUST_LAUNCH_MID,
        LAUNCH_MID,
        WAIT_FOR_LAUNCH_MID,
        SHOOT_MID_TO_START_FAR,
        START_FAR_TO_END_FAR,
        END_FAR_TO_END,
        ADJUST_LAUNCH_FAR,
        LAUNCH_FAR,
        WAIT_FOR_LAUNCH_FAR,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(124.8, 119.2, Math.toRadians(36));
    private final Pose closeShoot = new Pose(107.4, 106.6, Math.toRadians(45));
    private final Pose startClose = new Pose(83, 83, Math.toRadians(175));
    private final Pose endClose = new Pose(120.8, 83, Math.toRadians(180));
    private final Pose middleShoot = new Pose(85.5, 84.6, Math.toRadians(45));
    private final Pose startMid = new Pose(85, 54, Math.toRadians(180));
    private final Pose endMid = new Pose(122, 54, Math.toRadians(180));
    private final Pose startFar = new Pose(101.4, 37, Math.toRadians(180));
    private final Pose endFar = new Pose(122, 37, Math.toRadians(180));
    private final Pose endPose = new Pose(85.6, 105, Math.toRadians(40));


    private PathChain startToCloseShoot, closeShootToStartClose, startCloseToEndClose, endCloseToMiddleShoot, middleShootToStartMid, startMidToEndMid, endMidToMiddleShoot, middleShootToStartFar, startFarToEndFar, endFarToEnd;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, closeShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), closeShoot.getHeading())
                .build();
        closeShootToStartClose = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot, startClose))
                .setLinearHeadingInterpolation(closeShoot.getHeading(), startClose.getHeading())
                .build();
        startCloseToEndClose = follower.pathBuilder()
                .addPath(new BezierLine(startClose, endClose))
                .setLinearHeadingInterpolation(startClose.getHeading(), endClose.getHeading())
                .build();
        endCloseToMiddleShoot = follower.pathBuilder()
                .addPath(new BezierLine(endClose, middleShoot))
                .setLinearHeadingInterpolation(endClose.getHeading(), middleShoot.getHeading())
                .build();
        middleShootToStartMid = follower.pathBuilder()
                .addPath(new BezierLine(middleShoot, startMid))
                .setLinearHeadingInterpolation(middleShoot.getHeading(), startMid.getHeading())
                .build();
        startMidToEndMid = follower.pathBuilder()
                .addPath(new BezierLine(startMid, endMid))
                .setLinearHeadingInterpolation(startMid.getHeading(), endMid.getHeading())
                .build();
        endMidToMiddleShoot = follower.pathBuilder()
                .addPath(new BezierLine(endMid, middleShoot))
                .setLinearHeadingInterpolation(endMid.getHeading(), middleShoot.getHeading())
                .build();
        middleShootToStartFar = follower.pathBuilder()
                .addPath(new BezierLine(middleShoot, startFar))
                .setLinearHeadingInterpolation(middleShoot.getHeading(), startFar.getHeading())
                .build();
        startFarToEndFar = follower.pathBuilder()
                .addPath(new BezierLine(startFar, endFar))
                .setLinearHeadingInterpolation(startFar.getHeading(), endFar.getHeading())
                .build();
        endFarToEnd = follower.pathBuilder()
                .addPath(new BezierLine(endFar, endPose))
                .setLinearHeadingInterpolation(endFar.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case START:
                //check if follower is done with path
                //and check that 5 seconds has elapsed
                robotHardware.resetMechanisms();
                robotHardware.setFlywheelSpeedFrontPosition();
                robotHardware.setNormalTrajectory();

                telemetry.addLine("Path State: Start");
                setPathState(PathState.START_TO_PRELOAD_SHOOT);
                break;
            case START_TO_PRELOAD_SHOOT:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start to preload shoot");
                    follower.followPath(startToCloseShoot, 0.4, true);
                    setPathState(PathState.LAUNCH_PRELOAD);
                }
                break;
            case LAUNCH_PRELOAD:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Launch preload");
                    robotHardware.launch(true);
                    setPathState(PathState.WAIT_FOR_LAUNCH_PRELOAD);
                }
                break;
            case WAIT_FOR_LAUNCH_PRELOAD:
                telemetry.addLine("Path State: Wait for launch preload");
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_PRELOAD);
                        robotHardware.startIntake();
                    } else {
                        setPathState(PathState.SHOOT_PRELOAD_TO_START_CLOSE);
                        robotHardware.stopSpin();
                        robotHardware.stopIntake();
                        shotsToFire = 5;
                    }
                }
                break;
            case SHOOT_PRELOAD_TO_START_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot preload to start close");
                    follower.followPath(closeShootToStartClose, .8, true);

                    robotHardware.setFlywheelSpeedMiddlePosition();
                    robotHardware.startIntake();
                    robotHardware.startSpin();

                    setPathState(PathState.START_CLOSE_TO_END_CLOSE);
                }
                break;
            case START_CLOSE_TO_END_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start close to end close");
                    follower.followPath(startCloseToEndClose, .6, true);
                    setPathState(PathState.END_CLOSE_TO_SHOOT_CLOSE);
                }
                break;
            case END_CLOSE_TO_SHOOT_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: End close to shoot close");
                    follower.followPath(endCloseToMiddleShoot, .8, true);

//                    robotHardware.stopIntake();
                    robotHardware.stopSpin();

                    setPathState(PathState.ADJUST_LAUNCH_CLOSE);
                }
                break;
            case ADJUST_LAUNCH_CLOSE:
                if (!follower.isBusy()){
                    robotHardware.adjustLauncherRed();
                    setPathState(PathState.LAUNCH_CLOSE);
                }
            case LAUNCH_CLOSE:
                if (!follower.isBusy()){
                    robotHardware.adjustLauncherRed();
                    telemetry.addLine("Path State: Launch close");
                    robotHardware.launch(true);
                    setPathState(PathState.WAIT_FOR_LAUNCH_CLOSE);
                }
                break;
            case WAIT_FOR_LAUNCH_CLOSE:
                telemetry.addLine("Path State: Wait for launch close");
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_CLOSE);
                    } else {
                        setPathState(PathState.SHOOT_CLOSE_TO_START_MID);
                        robotHardware.stopSpin();
                        shotsToFire = 5;
                    }
                }
                break;
            case SHOOT_CLOSE_TO_START_MID:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot close to start mid");
                    follower.followPath(middleShootToStartMid, 0.8, true);

//                    robotHardware.startIntake();
                    robotHardware.startSpin();

                    setPathState(PathState.START_MID_TO_END_MID);
                }
                break;
            case START_MID_TO_END_MID:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start mid to end mid");
                    follower.followPath(startMidToEndMid, 0.6, true);
                    setPathState(PathState.END_MID_TO_SHOOT_MID);
                }
                break;
            case END_MID_TO_SHOOT_MID:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: End mid to shoot mid");
                    follower.followPath(endMidToMiddleShoot, 0.8, true);

//                    robotHardware.stopIntake();
                    robotHardware.stopSpin();

                    setPathState(PathState.ADJUST_LAUNCH_MID);
                }
            case ADJUST_LAUNCH_MID:
                if (!follower.isBusy()){
                    robotHardware.adjustLauncherRed();
                    setPathState((PathState.LAUNCH_MID));
                }
            case LAUNCH_MID:
                if (!follower.isBusy()) {
                    robotHardware.adjustLauncherRed();
                    telemetry.addLine("Path State: Launch mid");
                    robotHardware.launch(true);
                    setPathState(PathState.WAIT_FOR_LAUNCH_MID);
                }
                break;
            case WAIT_FOR_LAUNCH_MID:
                telemetry.addLine("Path State: Wait for launch mid");
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_MID);
                    } else {
                        setPathState(PathState.SHOOT_MID_TO_START_FAR);
                        robotHardware.stopSpin();
                        shotsToFire = 5;
                    }
                }
                break;
            case SHOOT_MID_TO_START_FAR:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot mid to start far");
                    follower.followPath(middleShootToStartFar, .8, true);

//                    robotHardware.startIntake();
                    robotHardware.startSpin();

                    setPathState(PathState.START_FAR_TO_END_FAR);
                }
                break;
            case START_FAR_TO_END_FAR:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start far to end far");
                    follower.followPath(startFarToEndFar, .6, true);
                    setPathState(PathState.END_FAR_TO_END);
                }
                break;
            case END_FAR_TO_END:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot preload to start close");
                    follower.followPath(endFarToEnd, 1, true);

//                    robotHardware.stopIntake();
                    robotHardware.stopSpin();

                    setPathState(PathState.ADJUST_LAUNCH_FAR);
                }
                break;
            case ADJUST_LAUNCH_FAR:
                if (!follower.isBusy()){
                    robotHardware.adjustLauncherRed();
                    setPathState(PathState.LAUNCH_FAR);
                }
            case LAUNCH_FAR:
                if (!follower.isBusy()){
                    robotHardware.adjustLauncherRed();
                    telemetry.addLine("Path State: Launch far");
                    robotHardware.launch(true);
                    setPathState(PathState.WAIT_FOR_LAUNCH_FAR);
                }
                break;
            case WAIT_FOR_LAUNCH_FAR:
                telemetry.addLine("Path State: Wait for launch far");
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_FAR);
                    } else {
                        setPathState(PathState.DONE);
                        robotHardware.stopSpin();
                        shotsToFire = 5;
                    }
                }
                break;
            case DONE:
                telemetry.addLine("Done with complete auto");
                robotHardware.stopFlywheel();
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
        pathState = PathState.START;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        hardwareInit();
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
