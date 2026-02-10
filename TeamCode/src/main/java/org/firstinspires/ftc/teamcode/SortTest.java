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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SortTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    int shotsToFire = 4;

    int patternNum;

    RobotHardwareV2 robotHardware;

    private double fudgeFactor = Math.toRadians(15);

    int[] order123 = new int[]{1, 2, 3};
    int[] order132 = new int[]{1, 3, 2};
    int[] order213 = new int[]{2, 1, 3};
    int[] order231 = new int[]{2, 3, 1};
    int[] order321 = new int[]{3, 2, 1};
    int[] order312 = new int[]{3, 1, 2};

    int[] launchOrder;

    public void hardwareInit(){
        robotHardware = new RobotHardwareV2();
        RobotInitializerV2.initializeRobot(hardwareMap, robotHardware);

        robotHardware.resetMechanismsUp();
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
        LAUNCH_CLOSE,
        WAIT_FOR_LAUNCH_CLOSE,
        SHOOT_CLOSE_TO_START_MID,
        START_MID_TO_END_MID,
        END_MID_TO_SHOOT_MID,
        LAUNCH_MID,
        WAIT_FOR_LAUNCH_MID,
        SHOOT_MID_TO_START_FAR,
        START_FAR_TO_END_FAR,
        END_FAR_TO_END,
        LAUNCH_FAR,
        WAIT_FOR_LAUNCH_FAR,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(19.2, 119.1, Math.toRadians(144));
    private final Pose detectAprilTag = new Pose(36.6, 106.6, Math.toRadians(85));
    private final Pose closeShoot = new Pose(37, 105, Math.toRadians(145));
    private final Pose startClose = new Pose(52, 78, Math.toRadians(5));//(42, 88.1, Math.toRadians(5));
    private final Pose endClose = new Pose(23, 78, Math.toRadians(0));//(15.4, 76.6, Math.toRadians(0));
    private final Pose middleShoot = new Pose(58.7, 84.6, Math.toRadians(139));
    private final Pose startMid = new Pose(43.9, 52, Math.toRadians(0));//(41.9, 57.5, Math.toRadians(0));
    private final Pose endMid = new Pose(17, 52, Math.toRadians(0));//(10.4, 57.5, Math.toRadians(0));
    private final Pose startFar = new Pose(38, 30, Math.toRadians(0));//(41.6, 33.8, Math.toRadians(0));
    private final Pose endFar = new Pose(15, 30, Math.toRadians(0));//(11.1, 33.8, Math.toRadians(0));
    private final Pose endPose = new Pose(59.1, 105, Math.toRadians(145));


    private PathChain startToDetectTag, detectTagToCloseShoot, closeShootToStartClose, startCloseToEndClose, endCloseToMiddleShoot, middleShootToStartMid, startMidToEndMid, endMidToMiddleShoot, middleShootToStartFar, startFarToEndFar, endFarToEnd;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToDetectTag = follower.pathBuilder()
                .addPath(new BezierLine(startPose, detectAprilTag))
                .setLinearHeadingInterpolation(startPose.getHeading(), detectAprilTag.getHeading())
                .build();
        detectTagToCloseShoot = follower.pathBuilder()
                .addPath(new BezierLine(detectAprilTag, closeShoot))
                .setLinearHeadingInterpolation(detectAprilTag.getHeading(), closeShoot.getHeading())
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
                robotHardware.resetMechanismsUp();
                robotHardware.setFlywheelSpeedFrontPosition();

                telemetry.addLine("Path State: Start");
                setPathState(PathState.START_TO_PRELOAD_SHOOT);
                break;
            case START_TO_PRELOAD_SHOOT:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start to preload shoot");
                    robotHardware.setRedAngle();
                    follower.followPath(startToDetectTag, 0.5, true);

                    setPathState(PathState.LAUNCH_PRELOAD);
                }
                break;
            case LAUNCH_PRELOAD:
                if (patternNum == 0){
                    int aprilTagID = robotHardware.getDetectedAprilTag(telemetry);
                    if (aprilTagID != 0 ){
                        patternNum = aprilTagID;
                    }
                }

                if (!follower.isBusy()){
                    if (patternNum != 0) { // detected pattern

                        if (patternNum == 21) { // GPP
                            launchOrder = order213;
                        } else if (patternNum == 22) { // PGP
                            launchOrder = order123;
                        } else {
                            launchOrder = order132; // PPG
                        }
                        robotHardware.resetLaunchAndSortState();
                        telemetry.addLine("Path State: Launch preload. order: " + launchOrder);
                        follower.followPath(detectTagToCloseShoot, 0.4, true);
                        setPathState(PathState.WAIT_FOR_LAUNCH_PRELOAD);
                    }
                }
                break;
            case WAIT_FOR_LAUNCH_PRELOAD:
                telemetry.addLine("Path State: Wait for launch preload");
                robotHardware.setAngleStraight();

                if (!follower.isBusy()) {
                    if (robotHardware.sortAndLaunch(true, launchOrder, telemetry)) {
                        setPathState(PathState.SHOOT_PRELOAD_TO_START_CLOSE);
                        robotHardware.stopSpin();
                    }
                }
                break;
            case SHOOT_PRELOAD_TO_START_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot preload to start close");
                    follower.followPath(closeShootToStartClose, .8, true);

                    robotHardware.resetMechanismsMiddle();
                    robotHardware.setFlywheelSpeedMiddlePosition();
                    robotHardware.startIntake();
                    robotHardware.startSpin();

                    setPathState(PathState.START_CLOSE_TO_END_CLOSE);
                }
                break;
            case START_CLOSE_TO_END_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Start close to end close");
                    follower.followPath(startCloseToEndClose, .5, true);
                    setPathState(PathState.END_CLOSE_TO_SHOOT_CLOSE);
                }
                break;
            case END_CLOSE_TO_SHOOT_CLOSE:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: End close to shoot close");
                    follower.followPath(endCloseToMiddleShoot, .8, true);
                    robotHardware.resetMechanismsUp();

//                    robotHardware.stopIntake();
                    robotHardware.stopSpin();

                    setPathState(PathState.LAUNCH_CLOSE);
                }
                break;
            case LAUNCH_CLOSE:
                if (!follower.isBusy()){
                    if (patternNum != 0) { // detected pattern
                        if (patternNum == 21) { // GPP
                            launchOrder = order312;
                        } else if (patternNum == 22) { // PGP
                            launchOrder = order132;
                        } else {
                            launchOrder = order123; // PPG
                        }
                        robotHardware.resetLaunchAndSortState();
                        telemetry.addLine("Path State: Launch close. order: " + launchOrder);
                        setPathState(PathState.WAIT_FOR_LAUNCH_CLOSE);
                    }
                }
                break;
            case WAIT_FOR_LAUNCH_CLOSE:
                telemetry.addLine("Path State: Wait for launch close");
                robotHardware.setAngleStraight();

                if (!follower.isBusy()) {
                    if (robotHardware.sortAndLaunch(true, launchOrder, telemetry)) {
                        setPathState(PathState.SHOOT_CLOSE_TO_START_MID);
                        robotHardware.stopSpin();
                    }
                }
                break;
            case SHOOT_CLOSE_TO_START_MID:
                if (!follower.isBusy()){
                    telemetry.addLine("Path State: Shoot close to start mid");
                    follower.followPath(middleShootToStartMid, 0.8, true);

                    robotHardware.resetMechanismsMiddle();
                    robotHardware.startIntake();
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
                    robotHardware.resetMechanismsUp();

//                    robotHardware.stopIntake();
                    robotHardware.stopSpin();

                    setPathState(PathState.LAUNCH_MID);
                }
            case LAUNCH_MID:
                if (!follower.isBusy()){
                    if (patternNum != 0) { // detected pattern
                        if (patternNum == 21) { // GPP
                            launchOrder = order213;
                        } else if (patternNum == 22) { // PGP
                            launchOrder = order123;
                        } else {
                            launchOrder = order132; // PPG
                        }
                        robotHardware.resetLaunchAndSortState();
                        telemetry.addLine("Path State: Launch mid. order: " + launchOrder);
                        setPathState(PathState.WAIT_FOR_LAUNCH_MID);
                    }
                }
                break;
            case WAIT_FOR_LAUNCH_MID:
                telemetry.addLine("Path State: Wait for launch mid");
                robotHardware.setAngleStraight();

                if (!follower.isBusy()) {
                    if (robotHardware.sortAndLaunch(true, launchOrder, telemetry)) {
                        setPathState(PathState.DONE);
                        robotHardware.stopSpin();
                    }
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
//
//        telemetry.addData("path state", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("April Tag Num: ", patternNum);

    }


}
