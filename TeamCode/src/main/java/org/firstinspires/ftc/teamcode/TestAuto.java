package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name="TestAuto", group="Linear Opmode")
public class TestAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armM;
    private Servo leftClawS;
    private Servo rightClawS;
    private Servo wristS;
    private Servo armS;

    private int newArmTarget;
    public static double ROBOT_WIDTH_INCHES = 16.2;
    public static double ROBOT_LENGTH_INCHES = 12;


    @Override
    public void runOpMode() throws InterruptedException {
        armM = hardwareMap.get(DcMotor.class, "armM");
        leftClawS = hardwareMap.get(Servo.class, "leftClawS");
        rightClawS = hardwareMap.get(Servo.class, "rightClawS");
        wristS = hardwareMap.get(Servo.class, "wristS");
        armS = hardwareMap.get(Servo.class, "armS");

        armM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armM.setDirection(DcMotor.Direction.FORWARD);

        armM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double START_POSITION_X = -1 - (ROBOT_WIDTH_INCHES / 2);
        double START_POSITION_Y = -72 + (ROBOT_LENGTH_INCHES / 2);
        // start position of red side, left of center
        Pose2d startPose = new Pose2d(START_POSITION_X, START_POSITION_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d latestPose = startPose;
        latestPose = hangSpecimen(36 - ROBOT_LENGTH_INCHES / 2, 13, drive, latestPose);
        latestPose = pickUpSample(drive, latestPose);
        latestPose = placeSample(18, 8, drive, latestPose);
        latestPose = pickUpSample2(drive, latestPose);
        latestPose = placeSample(18, 8, drive, latestPose);
        latestPose = pickUpSample3(drive, latestPose);
        latestPose = placeSample(18, 8, drive, latestPose);
        //gotoAscentZone(20, 39, 12, drive, latestPose);
    }

    public Pose2d pickUpSample3(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .build();
        TrajectorySequence toSample = drive.trajectorySequenceBuilder(backTraj.end())
                .turn(Math.toRadians(180) - backTraj.end().getHeading())
                .lineToConstantHeading(new Vector2d(-50, -26))
                .build();
        TrajectorySequence pushSample = drive.trajectorySequenceBuilder(toSample.end())
                .forward(10)
                .back(7)
                .build();
        TrajectorySequence grabSample = drive.trajectorySequenceBuilder(pushSample.end())
                .forward(9)
                .build();
        drive.followTrajectorySequence(backTraj);

        //arm down
        armDown();
        wristUp();

        drive.followTrajectorySequence(toSample);

        closeClaw();
        wristS.setPosition(.65);

        drive.followTrajectorySequence(pushSample);

        openClaw();


        drive.followTrajectorySequence(grabSample);

        closeClaw();

        return grabSample.end();
    }

    public Pose2d pickUpSample2(SampleMecanumDrive drive, Pose2d startPose){
            TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(18)
                .build();
            TrajectorySequence goToSample = drive.trajectorySequenceBuilder(backTraj.end())
                    .turn(Math.toRadians(90)-backTraj.end().getHeading())
                    .lineToConstantHeading(new Vector2d (-61, -42))
                    .build();
            drive.followTrajectorySequence(backTraj);

            //arm down
            armDown();
            wristUp();

            drive.followTrajectorySequence(goToSample);

            wristS.setPosition(.6);
            sleep(250);
            closeClaw();

            return goToSample.end();

    };

    public Pose2d gotoAscentZone(double backDist, double leftDist, double fwdDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(backDist)
                .build();
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(backTraj.end());
        // builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence leftTraj = builder
                .turn(Math.toRadians(15) - backTraj.end().getHeading())
                .splineToLinearHeading(new Pose2d(-16 - ROBOT_LENGTH_INCHES / 2, -12), Math.toRadians(0))
                /*
                .turn(Math.toRadians(0) - backTraj.end().getHeading())
                .strafeLeft(leftDist)
                .forward(fwdDist)
                 */
                .build();
        drive.followTrajectorySequence(backTraj);
       // moveArm(0.2, 5, "lowerArm");
        sleep(500);
        drive.followTrajectorySequence(leftTraj);
        return leftTraj.end();

    }

    public Pose2d hangSpecimen(double fwdDist, double backDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(fwdDist)
                //.lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .back(backDist)
                .build();
       // moveArm(0.1, 8, "raiseArm");
        wrist90();
        closeClaw();
        armDown();
        sleep(2150);

        drive.followTrajectorySequence(forwardTraj);

        //moveArm(0.4, 4, "lowerArm");
        sleep(2000);

        drive.followTrajectorySequence(backTraj);

        openClaw();
        wristUp();

        return backTraj.end();

    }

    public Pose2d pickUpSample(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        // builder.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(6));
        TrajectorySequence leftTraj = builder
                .lineToConstantHeading(new Vector2d(-51 - 1, -43))
                //.forward(6) // fudge factor
                .build();
        TrajectorySequence fwdTraj = drive.trajectorySequenceBuilder(leftTraj.end())
                .lineToConstantHeading(new Vector2d(-48 - 2, -24))
                .build();
        //moveArm(0.3, 9, "lowerArm");
        sleep(1000);

        drive.followTrajectorySequence(leftTraj);
        //drive.followTrajectorySequence(fwdTraj);

        wristS.setPosition(.6);
        sleep(500);
        closeClaw();

        // return fwdTraj.end();
        return leftTraj.end();
    }

    public Pose2d placeSample(double backDist, double fwdDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence toBasketTraj = drive.trajectorySequenceBuilder(startPose)
                // .back(backDist)
                .lineToConstantHeading(new Vector2d(-48, -48))
                .turn(Math.toRadians(180 + 45 + 3) - startPose.getHeading())
                .build();
        TrajectorySequence fwdTraj = drive.trajectorySequenceBuilder(toBasketTraj.end())
                // .forward(fwdDist)
                .lineToConstantHeading(new Vector2d(-53, -53))
                .build();
        drive.followTrajectorySequence(toBasketTraj);

        //moveArm(0.4, 10, "raiseArm");
        armMid();
        wrist180();
        sleep(2500);

        drive.followTrajectorySequence(fwdTraj);

        openClaw();

        return fwdTraj.end();
    }
/*
    private void drive(double speed, int leftTarget, int rightTarget, String direction) {

        telemetry.addData("Drive", direction);
        telemetry.update();
    }
*/


    private void reset() {

        armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArm(double speed, int armTarget, String direction) {
        reset();
        armTarget *= 2;

        newArmTarget = armTarget;

        if (direction == "raiseArm") {
            armM.setTargetPosition(-newArmTarget);

        } else if (direction == "lowerArm") {
            armM.setTargetPosition(newArmTarget);
        }

        armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armM.setPower(speed);

        sleep(300);
    }

    //claw commands

    public void closeClaw() {      //close
        leftClawS.setPosition(.67);
        rightClawS.setPosition(.33);
    }

    public void openClaw() {       //open
        leftClawS.setPosition(.82);
        rightClawS.setPosition(.18);
    }

    //wrist commands

    public void wristDown() {      //down
        wristS.setPosition(0);
    }

    public void wristUp() {        //up
        wristS.setPosition(1);
    }

    public void wrist180() {       //halfway
        wristS.setPosition(0.45);
    }

    public void wrist90() {
        wristS.setPosition(.75);
    }

    //arm commands

    public void armUp() {          //up
        armS.setPosition(0.115);
    }

    public void armDown() {        //down
        armS.setPosition(0);
    }

    public void armMid() {         //halfway
        armS.setPosition(0.1);
    }
/*
    public void armLow() {         //-45 degrees
        armS.setPosition(0.6);
    }

 */
}