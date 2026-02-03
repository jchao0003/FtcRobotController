package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous
public class autoRedBack extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotorEx flywheel;
    private DcMotor intake;

    RobotHardware robotHardware;




    private double fudgeFactor = Math.toRadians(10);

    private Servo feeder;

    public void launch3(){
        robotHardware.launch();
        sleep(1000);
        robotHardware.launchPt2();
        sleep(500);
        robotHardware.launch();
        sleep(1000);
        robotHardware.launchPt2();
        sleep(750);
        robotHardware.launch();
        sleep(1000);
        robotHardware.launchPt2();
        sleep(750);
        robotHardware.launch();
    }


    public void hardwareInit(){
        robotHardware = new RobotHardware();
        RobotInitializer.initializeRobot(hardwareMap, robotHardware);

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

        SHOOT_PRELOAD,

        DRIVE_TO_PRESET,

        PICKUP_PRESET,

        PRESET_TO_SHOOT,

        SHOOT_PRESET,

        DRIVE_TO_PRESET2,

        PICKUP_PRESET2,

        PRESET2_TO_SHOOT,

        SHOOT_PRESET2,

        MOVE_OUT_OF_LAUNCH,

        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose farShootPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose farPresetStart = new Pose(90, 32.5, Math.toRadians(180));
    private final Pose farPresetEnd = new Pose(133, 32.5, Math.toRadians(180));
    private final Pose middlePresetStart = new Pose(90, 56, Math.toRadians(180));
    private final Pose middlePresetEnd = new Pose(133, 56, Math.toRadians(180));


    private PathChain startToFarPresetStart, farPresetStartToFarPresetEnd, farPresetEndToShoot, shootToMiddlePresetStart, middlePresetStartToMiddlePresetEnd, middlePresetEndToShoot, shootToOutOfLaunch;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToFarPresetStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farPresetStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), farPresetStart.getHeading())
                .build();
        farPresetStartToFarPresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(farPresetStart, farPresetEnd))
                .setLinearHeadingInterpolation(farPresetStart.getHeading(), farPresetEnd.getHeading())
                .setVelocityConstraint(1)
                .build();
        farPresetEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(farPresetEnd, farShootPose))
                .setLinearHeadingInterpolation(farPresetEnd.getHeading(), farShootPose.getHeading()-fudgeFactor)
                .build();
        shootToMiddlePresetStart = follower.pathBuilder()
                .addPath(new BezierLine(farShootPose, middlePresetStart))
                .setLinearHeadingInterpolation(farShootPose.getHeading(), middlePresetStart.getHeading()+fudgeFactor)
                .build();
        middlePresetStartToMiddlePresetEnd = follower.pathBuilder()
                .addPath(new BezierLine(middlePresetStart, middlePresetEnd))
                .setLinearHeadingInterpolation(middlePresetStart.getHeading(), middlePresetEnd.getHeading())
                .setVelocityConstraint(1)
                .build();
        middlePresetEndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(middlePresetEnd, farShootPose))
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case SHOOT_PRELOAD:
                //check if follower is done with path
                //and check that 5 seconds has elapsed

                robotHardware.resetMechanisms();
                robotHardware.launchPt2();
                robotHardware.setFlywheelSpeedBackPosition();
                robotHardware.setRedAngle();
                sleep(4000);

                if (!follower.isBusy()){
                    launch3();
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.DRIVE_TO_PRESET);
                }
                break;
            case DRIVE_TO_PRESET:
                //all done!
                if (!follower.isBusy()){
                    telemetry.addLine("To preload");
                    follower.followPath(startToFarPresetStart);
                    robotHardware.startIntake();
                    setPathState(PathState.PICKUP_PRESET);
                }
                break;
            case PICKUP_PRESET:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up preload");
                    follower.followPath(farPresetStartToFarPresetEnd);
                    setPathState(PathState.PRESET_TO_SHOOT);
                }
                break;
            case PRESET_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("To launch zone");
                    follower.followPath(farPresetEndToShoot);
                    robotHardware.stopIntake();
                    sleep(500);
                    robotHardware.startIntake();
                    setPathState(PathState.SHOOT_PRESET);
                }
                break;
            case SHOOT_PRESET:
                if(!follower.isBusy()){
                    robotHardware.stopIntake();
                    telemetry.addLine("Launching");
                    launch3();
                    setPathState(PathState.DRIVE_TO_PRESET2); //If want to do second preset line change this to DRIVE_TO_PRESET2
                }
                break;
            case DRIVE_TO_PRESET2:
                if(!follower.isBusy()){
                    telemetry.addLine("To middle preset");
                    follower.followPath(shootToMiddlePresetStart);
                    robotHardware.startIntake();
                    setPathState(PathState.PICKUP_PRESET2);
                }
                break;
            case PICKUP_PRESET2:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up middle preset");
                    follower.followPath(middlePresetStartToMiddlePresetEnd);
                    setPathState(PathState.PRESET2_TO_SHOOT);
                }
                break;
            case PRESET2_TO_SHOOT:
                if(!follower.isBusy()){
                    telemetry.addLine("To launch zone");
                    follower.followPath(middlePresetEndToShoot);
                    robotHardware.stopIntake();
                    sleep(500);
                    robotHardware.startIntake();
                    setPathState(PathState.SHOOT_PRESET2);
                }
                break;
            case SHOOT_PRESET2:
                if(!follower.isBusy()){
                    robotHardware.stopIntake();
                    telemetry.addLine("Launching middle preset");
                    launch3();
                    setPathState(PathState.MOVE_OUT_OF_LAUNCH);
                }
                break;
            case MOVE_OUT_OF_LAUNCH:
                if(!follower.isBusy()){
                    telemetry.addLine("Moving out of launch");
                    follower.followPath(startToFarPresetStart);
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
        pathState = PathState.SHOOT_PRELOAD;
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
