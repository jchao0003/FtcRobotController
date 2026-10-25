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
public class BiobuzzSecondLaunch extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotorEx flywheel;
    private DcMotor intake;
    int shotsToFire = 5;

    BiobuzzRobotHardware robotHardware;

    private double fudgeFactor = Math.toRadians(0);

    private ElapsedTime launchTimer = new ElapsedTime();

    private Servo feeder;

    final double launchTime = 5.0;




    public void hardwareInit(){
        robotHardware = new BiobuzzRobotHardware();
        BiobuzzRobotInitializer.initializeRobot(hardwareMap, robotHardware);

        intake = robotHardware.intake;
        flywheel = robotHardware.flywheel;

        robotHardware.resetMechanisms();

    }


    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT . ATTEMPT TO SCORE THE POLLEN

        START,
        LAUNCH_PRELOAD,
        MOVE_TO_LOADING_ZONE,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(60, 132, Math.toRadians(270));
    private final Pose midPathPoint = new Pose(58, 123, Math.toRadians(270));
    private final Pose endPose = new Pose(15, 119, Math.toRadians(270));


    private PathChain startToMidPathPoint, midPathPointToEndPose;

    public void buildPaths(){
        // put in coordinates for starting pose then coordinates for ending pose
        startToMidPathPoint = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPathPoint))
                .setLinearHeadingInterpolation(startPose.getHeading() + fudgeFactor, midPathPoint.getHeading())
                .build();
        midPathPointToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(midPathPoint,endPose))
                .setLinearHeadingInterpolation(midPathPoint.getHeading() + fudgeFactor,endPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case START:
                //check if follower is done with path
                //and check that 5 seconds has elapsed
                robotHardware.resetMechanisms();

                if (launchTimer.seconds() > 10){
                    robotHardware.setFlywheelVelocity();
                }

                if (!follower.isBusy()){
                    telemetry.addLine("Done with start");
                    if (launchTimer.seconds() > 15){
                        setPathState(PathState.LAUNCH_PRELOAD);
                    }
                }
                break;
            case LAUNCH_PRELOAD:
                if (!follower.isBusy()){
                    telemetry.addLine("Launching preload");
                    robotHardware.setFeedLaunch();
                    sleep(1000);
                    robotHardware.setFeedDown();
                    sleep(1500);
                    robotHardware.setFeedLaunch();
                    sleep(1000);
                    robotHardware.setFeedDown();
                    sleep(1500);
                    robotHardware.setFeedLaunch();
                    sleep(1000);
                    robotHardware.setFeedDown();
                    sleep(1500);
                    robotHardware.setFeedLaunch();
                    sleep(1000);
                    robotHardware.setFeedDown();
                    sleep(1500);
//                    robotHardware.setFeedLaunch();
//                    sleep(1000);
//                    robotHardware.setFeedDown();
//                    sleep(1000);
//                    launchTimer.reset();
//                    if (shotsToFire > 0){
//                        if (launchTimer.seconds() > launchTime) {
//                            robotHardware.setFeedLaunch();
//                            launchTimer.reset();
//                        } else {
//                            robotHardware.setFeedDown();
//                            shotsToFire--;
//                        }
//                    }
                    setPathState(PathState.MOVE_TO_LOADING_ZONE);
                }
                break;
            case MOVE_TO_LOADING_ZONE:
                //all done!
                if (!follower.isBusy()){
                    telemetry.addLine("To loading zone");
                    follower.followPath(startToMidPathPoint);
                    follower.followPath(midPathPointToEndPose);
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
