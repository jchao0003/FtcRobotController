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

@Autonomous
public class SampleAutoPath extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

//    private DcMotorEx flywheel;
//    private DcMotorEx flywheel2;
//    private DcMotor intakeM;
//
//    private double velocity = 1000;
//
//    private Servo gateS;

//    double resultMaxVelocityTest = 2120.0;
//    double F = 32767.0/resultMaxVelocityTest;
//    double kP = F * 0.1;
//    double kI = kP * 0.1;
//    double kD = kI * 0.1;
//    double position = 5.0;


    public void hardwareInit(){
//        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
//        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
//        intakeM = hardwareMap.get(DcMotor.class, "intakeM");
//
//        gateS = hardwareMap.get(Servo.class, "gateS");
//
//        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        intakeM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
//        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
//        intakeM.setDirection(DcMotor.Direction.REVERSE);
//
//        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        flywheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        flywheel.setVelocityPIDFCoefficients(kP,kI,kD,F);
//        flywheel2.setVelocityPIDFCoefficients(kP,kI,kD,F);
//
//        flywheel.setPositionPIDFCoefficients(position);
//        flywheel2.setPositionPIDFCoefficients(position);

    }


//
//    public void launch3(){
//        gateS.setPosition(1);
//        sleep(500);
//        gateS.setPosition(0.8);
//        sleep(500);
//        gateS.setPosition(1);
//        sleep(500);
//        gateS.setPosition(0.8);
//        sleep(500);
//        gateS.setPosition(1);
//        sleep(500);
//        gateS.setPosition(0.8);
//    }
//
//    public void startIntake(){
//        intakeM.setPower(0.9);
//    }
//
//    public void stopIntake(){
//        intakeM.setPower(0);
//    }
//    public void reset(){
//        flywheel.setVelocity(velocity);
//        flywheel2.setVelocity(velocity);
//        gateS.setPosition(0.8);
//        sleep(1000);
//    }
//
//    public void stopFlywheel(){
//        flywheel.setPower(0);
//        flywheel2.setPower(0);
//    }


    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT . ATTEMPT TO SCORE THE ARTIFACT

        START_TO_SHOOT,

        SHOOT_PRELOAD,

        DRIVE_TO_PRESET,

        PICKUP_PRESET,

        PRESET_TO_SHOOT,

        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(22, 120, Math.toRadians(135));
    private final Pose nearShootPose = new Pose(53.5, 89, Math.toRadians(135));
    private final Pose nearPresetStart = new Pose(49, 79, Math.toRadians(0));
    private final Pose nearPresetEnd = new Pose(24.5, 79, Math.toRadians(0));


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
                //reset();
                follower.followPath(startToShoot, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer and make new state
                break;
            case SHOOT_PRELOAD:
                //check if follower is done with path
                //and check that 5 seconds has elapsed
                if (!follower.isBusy()){
                    //launch3();
                    telemetry.addLine("Done Path 1");
                    follower.followPath(nearShootToNearPresetStart);
                    setPathState(PathState.DRIVE_TO_PRESET);
                }
                break;
            case DRIVE_TO_PRESET:
                //all done!
                if (!follower.isBusy()){
                    telemetry.addLine("To preload");
                    //startIntake();
                    follower.followPath(nearPresetStartToNearPresetEnd);
                    setPathState(PathState.PICKUP_PRESET);
                }
                break;
            case PICKUP_PRESET:
                if(!follower.isBusy()){
                    telemetry.addLine("Picking up preload");
                    //stopIntake();
                    follower.followPath(nearPresetEndToShoot);
                    setPathState(PathState.PRESET_TO_SHOOT);
                }
                break;
            case PRESET_TO_SHOOT:
                if(!follower.isBusy()){
                    //launch3();
                    telemetry.addLine("To launch, done");
                    setPathState(PathState.DONE);
                }
                break;
            case DONE:
                if(!follower.isBusy()){
                    //stopFlywheel();
                    telemetry.addLine("Done with complete auto");
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
        pathState = PathState.START_TO_SHOOT;
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
