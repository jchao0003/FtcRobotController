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
public class autoBlueBackV3a extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotorEx flywheel;
    private DcMotor intake;
    int shotsToFire = 3;


    RobotHardwareV2 robotHardware;




    private double fudgeFactor = Math.toRadians(15);

    private Servo feeder;
//
//    public void launch3(){
//        robotHardware.launch(true);
//        sleep(1000);
//        robotHardware.launchPt2();
//        sleep(750);
//        robotHardware.launch();
//        sleep(1250);
//        robotHardware.launchPt2();
//        sleep(750);
//        robotHardware.launch();
//        sleep(1250);
//        robotHardware.launchPt2();
//        sleep(750);
//        robotHardware.launch();
//    }



    public void hardwareInit(){
        robotHardware = new RobotHardwareV2();
        RobotInitializerV2.initializeRobot(hardwareMap, robotHardware);

        intake = robotHardware.intake;
        flywheel = robotHardware.flywheel;

        robotHardware.resetMechanisms();
        robotHardware.launchPt2();
    }


    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT . ATTEMPT TO SCORE THE ARTIFACT

        START_TO_SHOOT,

        START,
        LAUNCH_PRELOAD,
        WAIT_FOR_LAUNCH_PRELOAD,
        LAUNCH_CORNER,
        WAIT_FOR_LAUNCH_CORNER,
        LAUNCH_FAR,
        WAIT_FOR_LAUNCH_FAR,
        LAUNCH_MID,
        WAIT_FOR_LAUNCH_MID,
        DRIVE_TO_PRESET_CORNER,
        PICKUP_PRESET_CORNER,
        PRESET_CORNER_TO_SHOOT,
        SHOOT_PRESET_CORNER,

        DRIVE_TO_PRESET_FAR,

        PICKUP_PRESET_FAR,

        PRESET_FAR_TO_SHOOT,

        SHOOT_PRESET_FAR,

        DRIVE_TO_PRESET_MID,

        PICKUP_PRESET_MID,

        PRESET_MID_TO_SHOOT,

        SHOOT_PRESET_MID,

        MOVE_OUT_OF_LAUNCH,

        DONE
    }

    autoBlueBackV3a.PathState pathState;

    private final Pose startPose = new Pose(57.3, 9.2, Math.toRadians(90));
    private final Pose farShootPose = new Pose(57.3, 9.2, Math.toRadians(90));
    private final Pose cornerPresetStart = new Pose(10, 32.5, Math.toRadians(90));
    private final Pose cornerPresetEnd = new Pose(10, 12, Math.toRadians(90));
    private final Pose farPresetStart = new Pose(45, 33.2, Math.toRadians(0));
    private final Pose farPresetEnd = new Pose(15, 33.2, Math.toRadians(0));
    private final Pose middlePresetStart = new Pose(45, 57.2, Math.toRadians(0));
    private final Pose middlePresetEnd = new Pose(15, 57.2, Math.toRadians(0));


    private PathChain startToCornerPresetStart, cornerPresetStartToCornerPresetEnd,cornerPresetEndToFarLaunch, farLaunchToFarPresetStart, farPresetStartToFarPresetEnd, farPresetEndToShoot, shootToMiddlePresetStart, middlePresetStartToMiddlePresetEnd, middlePresetEndToShoot, shootToOutOfLaunch;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToCornerPresetStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, cornerPresetStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), cornerPresetStart.getHeading())
                .build();
        cornerPresetStartToCornerPresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(cornerPresetStart,cornerPresetEnd))
                .setLinearHeadingInterpolation(cornerPresetStart.getHeading(),cornerPresetEnd.getHeading())
                .setVelocityConstraint(0.5)
                .build();
        cornerPresetEndToFarLaunch = follower.pathBuilder()
                .addPath(new BezierLine(cornerPresetEnd, farShootPose))
                .setLinearHeadingInterpolation(cornerPresetEnd.getHeading(), farShootPose.getHeading()+fudgeFactor)
                .build();
        farLaunchToFarPresetStart = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, farPresetStart))
                .setLinearHeadingInterpolation(farShootPose.getHeading()+fudgeFactor, farPresetStart.getHeading())
                .build();
        farPresetStartToFarPresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(farPresetStart, farPresetEnd))
                .setLinearHeadingInterpolation(farPresetStart.getHeading(), farPresetEnd.getHeading())
                .setVelocityConstraint(1)
                .build();
        farPresetEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farPresetEnd, farShootPose))
                .setLinearHeadingInterpolation(farPresetEnd.getHeading(), farShootPose.getHeading()+fudgeFactor)
                .build();
        shootToMiddlePresetStart = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, middlePresetStart))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), middlePresetStart.getHeading()-fudgeFactor)
                .build();
        middlePresetStartToMiddlePresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(middlePresetStart, middlePresetEnd))
                .setLinearHeadingInterpolation(middlePresetStart.getHeading(), middlePresetEnd.getHeading())
                .setVelocityConstraint(1)
                .build();
        middlePresetEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(middlePresetEnd, farShootPose))
                .setLinearHeadingInterpolation(middlePresetEnd.getHeading(), farShootPose.getHeading()+fudgeFactor)
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case START:
                //check if follower is done with path
                //and check that 5 seconds has elapsed
                robotHardware.resetMechanisms();
                robotHardware.setFlywheelSpeedBackPosition();
                robotHardware.setBlueAngle();
                robotHardware.launchPt2();
                sleep(4000);

                if (!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.LAUNCH_PRELOAD);
                }
                break;
            case LAUNCH_PRELOAD:
                robotHardware.startSpin();
                robotHardware.launch(true);
                setPathState(PathState.WAIT_FOR_LAUNCH_PRELOAD);
                break;

            case WAIT_FOR_LAUNCH_PRELOAD:
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_PRELOAD);
                    } else {
                        setPathState(PathState.DRIVE_TO_PRESET_FAR);
                        shotsToFire = 3;
                    }
                }
                break;
            case DRIVE_TO_PRESET_FAR:
                //all done!
                if (!follower.isBusy()){
                    telemetry.addLine("To preload");
                    follower.followPath(farLaunchToFarPresetStart);
                    robotHardware.startIntake();
                    setPathState(PathState.PICKUP_PRESET_FAR);
                }
                break;
            case PICKUP_PRESET_FAR:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up preset far");
                    follower.followPath(farPresetStartToFarPresetEnd);
                    setPathState(PathState.PRESET_FAR_TO_SHOOT);
                }
                break;
            case PRESET_FAR_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("To launch zone");
                    follower.followPath(farPresetEndToShoot);
                    //robotHardware.stopIntake();
                    sleep(500);
                    robotHardware.startIntake();
                    setPathState(PathState.LAUNCH_FAR);
                }
                break;
//            case SHOOT_PRESET_FAR:
//                if(!follower.isBusy()){
//                    robotHardware.stopIntake();
//                    telemetry.addLine("Launching");
//                    robotHardware.startIntake();
//                    launch3();
//                    robotHardware.stopIntake();
//                    setPathState(PathState.DRIVE_TO_PRESET_CORNER); //If want to do second preset line change this to DRIVE_TO_PRESET2
//                }
//                break;
            case LAUNCH_FAR:
                robotHardware.launch(true);
                setPathState(PathState.WAIT_FOR_LAUNCH_FAR);
                break;

            case WAIT_FOR_LAUNCH_FAR:
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_FAR);
                    } else {
                        setPathState(PathState.DRIVE_TO_PRESET_CORNER);
                        shotsToFire = 3;
                    }
                }
                break;
            case DRIVE_TO_PRESET_CORNER:
                if(!follower.isBusy()){
                    telemetry.addLine("To preset corner");
                    follower.followPath(startToCornerPresetStart);
                    robotHardware.startIntake();
                    setPathState(PathState.PICKUP_PRESET_CORNER);
                }
                break;
            case PICKUP_PRESET_CORNER:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up preset corner");
                    follower.followPath(cornerPresetStartToCornerPresetEnd);
                    setPathState(PathState.PRESET_CORNER_TO_SHOOT);
                }
                break;
            case PRESET_CORNER_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("Preset corner to launch");
                    follower.followPath(cornerPresetEndToFarLaunch);
                    robotHardware.stopIntake();
                    sleep(500);
                    robotHardware.startIntake();
                    setPathState(PathState.LAUNCH_CORNER);
                }
                break;
//            case SHOOT_PRESET_CORNER:
//                if(!follower.isBusy()){
//                    robotHardware.stopIntake();
//                    telemetry.addLine("Launching preset corner");
//                    robotHardware.startIntake();
//                    launch3();
//                    robotHardware.stopIntake();
//                    setPathState(PathState.DRIVE_TO_PRESET_MID);
//                }
//                break;
            case LAUNCH_CORNER:
                robotHardware.launch(true);
                setPathState(PathState.WAIT_FOR_LAUNCH_CORNER);
                break;

            case WAIT_FOR_LAUNCH_CORNER:
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_CORNER);
                    } else {
                        setPathState(PathState.DRIVE_TO_PRESET_MID);
                        shotsToFire = 3;
                    }
                }
                break;
            case DRIVE_TO_PRESET_MID:
                if(!follower.isBusy()){
                    telemetry.addLine("To middle preset");
                    follower.followPath(shootToMiddlePresetStart);
                    robotHardware.startIntake();
                    setPathState(PathState.PICKUP_PRESET_MID);
                }
                break;
            case PICKUP_PRESET_MID:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up middle preset");
                    follower.followPath(middlePresetStartToMiddlePresetEnd);
                    setPathState(PathState.PRESET_MID_TO_SHOOT);
                }
                break;
            case PRESET_MID_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("To launch zone");
                    follower.followPath(middlePresetEndToShoot);
                    robotHardware.stopIntake();
                    sleep(500);
                    robotHardware.startIntake();
                    setPathState(PathState.LAUNCH_MID);
                }
                break;
//            case SHOOT_PRESET_MID:
//                if(!follower.isBusy()){
//                    robotHardware.stopIntake();
//                    telemetry.addLine("Launching middle preset");
//                    robotHardware.startIntake();
//                    launch3();
//                    robotHardware.stopIntake();
//                    setPathState(PathState.MOVE_OUT_OF_LAUNCH);
//                }
//                break;
            case LAUNCH_MID:
                robotHardware.launch(true);
                setPathState(PathState.WAIT_FOR_LAUNCH_MID);
                break;

            case WAIT_FOR_LAUNCH_MID:
                if(robotHardware.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        setPathState(PathState.LAUNCH_MID);
                    } else {
                        setPathState(PathState.MOVE_OUT_OF_LAUNCH);
                        shotsToFire = 3;
                    }
                }
                break;
            case MOVE_OUT_OF_LAUNCH:
                if(!follower.isBusy()){
                    telemetry.addLine("Moving out of launch");
                    follower.followPath(farLaunchToFarPresetStart);
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done with complete auto");
                    robotHardware.stopFlywheel();
                }
                break;
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
        pathState = PathState.START;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(farShootPose);

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
