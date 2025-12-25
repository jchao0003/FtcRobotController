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
public class autoRedBack extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private DcMotor intakeM;

    private double velocity = 1500;
    private double fudgeFactor = Math.toRadians(10);


    private Servo gateS;
    private Servo standS;


    double resultMaxVelocityTest = 2120.0;
    double F = 32767.0/resultMaxVelocityTest;
    double kP = F * 0.1;
    double kI = kP * 0.1;
    double kD = kI * 0.1;
    double position = 5.0;


    public void hardwareInit(){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intakeM = hardwareMap.get(DcMotor.class, "intakeM");

        gateS = hardwareMap.get(Servo.class, "gateS");
        standS = hardwareMap.get(Servo.class, "standS");

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        intakeM.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        flywheel.setVelocityPIDFCoefficients(kP,kI,kD,F);
        flywheel2.setVelocityPIDFCoefficients(kP,kI,kD,F);

        flywheel.setPositionPIDFCoefficients(position);
        flywheel2.setPositionPIDFCoefficients(position);

    }



    public void launch3(){
        gateS.setPosition(1);
        sleep(500);
        gateS.setPosition(0.8);
        sleep(750);
        gateS.setPosition(1);
        sleep(500);
        gateS.setPosition(0.8);
        sleep(750);
        gateS.setPosition(1);
        sleep(500);
        gateS.setPosition(0.8);
    }

    public void startIntake(){
        intakeM.setPower(1);
    }

    public void stopIntake(){
        intakeM.setPower(0);
    }
    public void reset(){
        flywheel.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
        gateS.setPosition(0.8);
        standS.setPosition(0.075);
        sleep(2250);

    }

    public void stopFlywheel(){
        flywheel.setPower(0);
        flywheel2.setPower(0);
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

    private final Pose startPose = new Pose(92, 9, Math.toRadians(70));
    private final Pose farShootPose = new Pose(92, 9, Math.toRadians(70));
    private final Pose farPresetStart = new Pose(90, 32.5, Math.toRadians(180));
    private final Pose farPresetEnd = new Pose(130, 32.5, Math.toRadians(180));
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

                reset();

                if (!follower.isBusy()){
                    reset();
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
                    startIntake();
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
                    stopIntake();
                    sleep(500);
                    startIntake();
                    setPathState(PathState.SHOOT_PRESET);
                }
                break;
            case SHOOT_PRESET:
                if(!follower.isBusy()){
                    stopIntake();
                    telemetry.addLine("Launching");
                    launch3();
                    setPathState(PathState.DRIVE_TO_PRESET2); //If want to do second preset line change this to DRIVE_TO_PRESET2
                }
                break;
            case DRIVE_TO_PRESET2:
                if(!follower.isBusy()){
                    telemetry.addLine("To middle preset");
                    follower.followPath(shootToMiddlePresetStart);
                    startIntake();
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
                    stopIntake();
                    sleep(500);
                    startIntake();
                    setPathState(PathState.SHOOT_PRESET2);
                }
                break;
            case SHOOT_PRESET2:
                if(!follower.isBusy()){
                    stopIntake();
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
                    stopFlywheel();
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
