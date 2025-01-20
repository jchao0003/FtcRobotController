package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name="LeftAutoM3DWV2", group="Linear Opmode")
public class LeftAutoM3DWV2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armM;
    private Servo leftClawS;
    private Servo rightClawS;
    private Servo wristS;
    private Servo armS;

    private int newArmTarget;
    public static double ROBOT_WIDTH_INCHES = 16.2;
    public static double ROBOT_LENGTH_INCHES = 12;

    private static int ARM_LENGTH_INCHES = 11;
    private static int ROBOT_HEIGHT_INCHES = 16;

    public static int CLAW_WAIT_TIME = 250;


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
        latestPose = pickUpSample1(drive, latestPose);
        latestPose = placeSample(drive, latestPose);
        latestPose = pickUpSample2(drive, latestPose);
        latestPose = placeSample(drive, latestPose);
        latestPose = pickUpSample3(drive, latestPose);
        latestPose = placeSample(drive, latestPose);
        //gotoAscentZone(20, 39, 12, drive, latestPose);
    }

    public Pose2d pickUpSample3(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence toSample = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90+30) - startPose.getHeading())
                .lineToConstantHeading(new Vector2d(-60, -37))
                .build();

        drive.followTrajectorySequenceAsync(toSample);
        moveArm(0.7, 42 + ROBOT_HEIGHT_INCHES, "lowerArm");
        arm0();
        wristUp();
        openClaw();
        drive.waitForIdle();
        sleep(250);
        wristS.setPosition(.65);
        sleep(500);
        closeClaw();
        return toSample.end();
    }

    public Pose2d pickUpSample2(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        //builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence goToSample = builder
                .turn(Math.toRadians(85) - startPose.getHeading())
                .lineToConstantHeading(new Vector2d(-60, -43))
                .build();

        drive.followTrajectorySequenceAsync(goToSample);
        moveArm(0.7, 42 + ROBOT_HEIGHT_INCHES, "lowerArm");
        wristUp();
        arm0();
        drive.waitForIdle();
        sleep(250);
        wristS.setPosition(.65);
        sleep(CLAW_WAIT_TIME);
        closeClaw();

        return goToSample.end();

    }

    ;

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
        moveArm(0.5, 28 - ROBOT_HEIGHT_INCHES + ARM_LENGTH_INCHES, "raiseArm");
        closeClaw();
        arm0();
        wristUp();
        sleep(1000);
        drive.followTrajectorySequenceAsync(forwardTraj);
        drive.waitForIdle();

        wrist90();

        moveArm(0.5, 2, "lowerArm");
        sleep(1000);

        drive.followTrajectorySequence(backTraj);

        openClaw();
        wristUp();

        return backTraj.end();

    }

    public Pose2d pickUpSample1(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        // builder.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(6));
        TrajectorySequence leftTraj = builder
                .lineToConstantHeading(new Vector2d(-49, -43))
                .build();
        moveArm(0.5, 26, "lowerArm");
        drive.followTrajectorySequenceAsync(leftTraj);
        //moveArm(0.3, 9, "lowerArm");
        drive.waitForIdle();

        wristS.setPosition(.65);
        sleep(CLAW_WAIT_TIME);
        closeClaw();

        return leftTraj.end();
    }

    public Pose2d placeSample(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence toBasketTraj = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .turn(Math.toRadians(40) - startPose.getHeading())
                .build();
        drive.followTrajectorySequenceAsync(toBasketTraj);
        moveArm(0.7, 42 - ROBOT_HEIGHT_INCHES, "raiseArm");
        arm270();
        wrist180();
        drive.waitForIdle();
        //sleep(500);
        openClaw();
        sleep(50);

        return toBasketTraj.end();
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
        armTarget *= 96;

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

    public void arm270() {          //parallel to floor backwards
        armS.setPosition(0.30);
    }

    public void arm0() {        //all the way down
        armS.setPosition(0);
    }

    public void arm90() {         //parallel to floor front
        armS.setPosition(0.1);
    }

    public void arm180() {         //perpendicular to floor
        armS.setPosition(0.2);

    }
}