package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.util.Hardware;


@Autonomous(name="RightAutoILT3Specimen", group="Linear Opmode")

public class RightAutoILT3Specimen extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armM;
    private Servo leftClawS;
    private Servo rightClawS;
    private Servo wristS;
    private Servo armS;

    private int newArmTarget;

    //private FirstVisionProcessor visionProcessor;
    //private VisionPortal visionPortal;
    public static double ROBOT_WIDTH_INCHES = 16.2;
    public static double ROBOT_LENGTH_INCHES = 12;
    public static int CLAW_WAIT_TIME = 250;
    private static int ARM_LENGTH_INCHES = 10;
    private static int ROBOT_HEIGHT_INCHES = 15;
  /*public void loop() {
      telemetry.addData("Identified", visionProcessor.getSelection());
  }*/

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

        double START_POSITION_X = 2 + (ROBOT_WIDTH_INCHES / 2);
        double START_POSITION_Y = -72 + (ROBOT_LENGTH_INCHES / 2);
        // start position of red side, right of center
        Pose2d startPose = new Pose2d(START_POSITION_X, START_POSITION_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d latestPose = startPose;
        latestPose = newHangSpecimen(drive, latestPose);
        latestPose = grabSample1(drive, latestPose);
        latestPose = dropSample(drive, latestPose);
        latestPose = newSpecimen1(drive, latestPose);
        latestPose = newSpecimen2(drive, latestPose);
        /*
        latestPose = grabSample2(drive, latestPose);
        latestPose = dropSample(drive, latestPose);
        latestPose = newSpecimen1(drive, latestPose);
        latestPose = newSpecimen2(drive, latestPose);
        latestPose = grabSample3(drive, latestPose);
        latestPose = dropSample(drive, latestPose);
         */
        //latestPose = newSpecimen2(drive, latestPose);

        //latestPose = push2Samples(drive, latestPose);
        //latestPose = hangGrabSpecimen(drive, latestPose);
        //latestPose = push1Sample(drive, latestPose);
    }

    public Pose2d newSpecimen2(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(3)
                .lineTo(new Vector2d(40, -70+9.5+(ROBOT_LENGTH_INCHES/2)))
                .lineTo(new Vector2d(40, -72+9.5+(ROBOT_LENGTH_INCHES/2)))
                .build();
        TrajectorySequence hangSpecimen = drive.trajectorySequenceBuilder(backTraj.end())
                .lineToConstantHeading(new Vector2d(11, -40 - ROBOT_LENGTH_INCHES/2))
                .lineTo(new Vector2d(11, -36 - ROBOT_LENGTH_INCHES/2))
                .build();

        drive.followTrajectorySequenceAsync(backTraj);
        sleep(1000);
        moveArm(0.7, 45, "lowerArm");
        arm270();
        openClaw();
        wrist180();
        drive.waitForIdle();

        //wristS.setPosition(.6);
        sleep(50);
        closeClaw();
        sleep(100);

        moveArm(0.5, 3, "raiseArm");

        sleep(100);

        drive.followTrajectorySequenceAsync(hangSpecimen);
        arm0();
        sleep(250);
        wrist90();
        sleep(750);
        moveArm(0.7, 20, "raiseArm");
        wristS.setPosition(.8);
        drive.waitForIdle();

        moveArm(0.8, 12, "raiseArm");
        sleep(500);



        return hangSpecimen.end();

    }

    /*
        public Pose2d newSpecimen2(SampleMecanumDrive drive, Pose2d startPose){
            TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(38, -66))
                    .build();
            TrajectorySequence hangSpecimen = drive.trajectorySequenceBuilder(backTraj.end())
                    .lineTo(new Vector2d(14, -36))
                    .build();
            TrajectorySequence clipSpecimen = drive.trajectorySequenceBuilder(hangSpecimen.end())
                    .lineTo(new Vector2d(14, -48))
                    .build();

            drive.followTrajectorySequenceAsync(backTraj);
            armS.setPosition(0.35);
            wristS.setPosition(.5);
            openClaw();
            drive.waitForIdle();

            closeClaw();
            sleep(250);

            moveArm(0.5, 3, "raiseArm");

            sleep(250);

            drive.followTrajectorySequenceAsync(hangSpecimen);
            arm0();
            wrist90();
            moveArm(0.5, 36, "raiseArm");
            drive.waitForIdle();

            moveArm(0.5, 4, "raiseArm");
            sleep(250);


            drive.followTrajectorySequence(clipSpecimen);
            openClaw();
            moveArm(0.5, 40, "lowerArm");
            wristUp();

            return clipSpecimen.end();

        }
    */
    public Pose2d newHangSpecimen(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(startPose.getX(), -32-ROBOT_LENGTH_INCHES/2))
                //.lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .back(4)
                .build();
        closeClaw();
        arm0();
        wristUp();

        drive.followTrajectorySequenceAsync(forwardTraj);
        moveArm(0.7, 18, "raiseArm");
        drive.waitForIdle();

        moveArm(0.7, 15, "lowerArm");
        sleep(475);
        openClaw();

        drive.followTrajectorySequenceAsync(backTraj);
        wristUp();
        drive.waitForIdle();

        return backTraj.end();

    }

    public Pose2d grabSample3(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence toSample = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90-25) - startPose.getHeading())
                .lineToConstantHeading(new Vector2d(59, -36))
                .build();

        drive.followTrajectorySequenceAsync(toSample);
        arm0();
        wristUp();
        openClaw();
        rightClawS.setPosition(.17);
        sleep(1000);
        wristS.setPosition(.65);
        drive.waitForIdle();
        sleep(500);
        closeClaw();
        return toSample.end();
    }

    public Pose2d newSpecimen1(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(38, -72+11+(ROBOT_LENGTH_INCHES/2)))
                .lineTo(new Vector2d(38, -72+9+(ROBOT_LENGTH_INCHES/2)))
                .build();
        TrajectorySequence hangSpecimen = drive.trajectorySequenceBuilder(backTraj.end())
                .lineToConstantHeading(new Vector2d(8, -50))
                .lineTo(new Vector2d(8, -36 - ROBOT_LENGTH_INCHES/2))
                .build();

        drive.followTrajectorySequenceAsync(backTraj);
        arm270();
        wrist180();
        openClaw();
        drive.waitForIdle();

        //wristS.setPosition(.6);
        sleep(50);
        closeClaw();
        sleep(150);

        moveArm(0.5, 3, "raiseArm");

        sleep(100);

        drive.followTrajectorySequenceAsync(hangSpecimen);
        arm0();
        sleep(250);
        wrist90();
        sleep(750);
        moveArm(0.7, 20, "raiseArm");
        wristS.setPosition(.8);
        drive.waitForIdle();

        moveArm(0.8, 10, "raiseArm");
        sleep(200);



        return hangSpecimen.end();

    }

    public Pose2d grabSample2(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence sampleGrab = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(59, -38))
                .build();
        drive.followTrajectorySequenceAsync(sampleGrab);
        arm0();
        sleep(250);
        closeClaw();
        sleep(250);
        wristUp();
        openClaw();
        drive.waitForIdle();
        sleep(250);
        //openClaw();
        wristS.setPosition(.65);
        sleep(400);
        closeClaw();

        return sampleGrab.end();


    }

    public Pose2d dropSample(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence dropSample = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(50, -52))
                .build();

        drive.followTrajectorySequenceAsync(dropSample);

        arm270();
        wrist180();

        drive.waitForIdle();
        sleep(1100);
        openClaw();



        return dropSample.end();
    }


    public Pose2d grabSample1(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence sampleGrab = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(49, -40))
                .build();

        drive.followTrajectorySequenceAsync(sampleGrab);
        moveArm(0.5, 26, "lowerArm");
        drive.waitForIdle();

        wristS.setPosition(.65);
        sleep(350);
        closeClaw();

        return sampleGrab.end();


    }
    public Pose2d hangGrabSpecimen(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence specimenGrab = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(38, -70 + ROBOT_LENGTH_INCHES / 2))
                .build();
        TrajectorySequence specimenHang = drive.trajectorySequenceBuilder(specimenGrab.end())
                //.back(6)
                //.turn(Math.toRadians(90) - specimenGrab.end().getHeading())
                .lineToLinearHeading(new Pose2d (38, -60, Math.toRadians(180)))
                // .splineToLinearHeading(new Pose2d(24 - ROBOT_WIDTH_INCHES, -36), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(24 - ROBOT_WIDTH_INCHES, -36, Math.toRadians(90)))
                //.lineToConstantHeading(new Vector2d(24 - ROBOT_WIDTH_INCHES, -36))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(specimenHang.end())
                .back(12)
                .build();

        drive.followTrajectorySequence(specimenGrab);

        closeClaw();
        //raise arm
        sleep(500);

        drive.followTrajectorySequence(specimenHang);

        //slides down
        wrist90();

        drive.followTrajectorySequence(backTraj);

        openClaw();

        return specimenHang.end();
    }


    public Pose2d push1Sample(SampleMecanumDrive drive, Pose2d startPose) {

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence pushTraj = builder
                //.back(8)
                .turn(Math.toRadians(270) - startPose.getHeading())
                .lineToConstantHeading(new Vector2d(33, -48))//left
                .lineToConstantHeading(new Vector2d(33, -16))//back
                .lineToConstantHeading(new Vector2d(45, -20))//left
                .lineToConstantHeading(new Vector2d(45, -60))//forward
                .lineToConstantHeading(new Vector2d(38, -48))//back
                .build();
        TrajectorySequence specimenGrab = drive.trajectorySequenceBuilder(pushTraj.end())
                .lineTo(new Vector2d(38,-70 + ROBOT_LENGTH_INCHES/2))
                .build();
        TrajectorySequence specimenHang = drive.trajectorySequenceBuilder(specimenGrab.end())
                .back(6)
                .turn(Math.toRadians(90) - specimenGrab.end().getHeading())
                // .splineToLinearHeading(new Pose2d(24 - ROBOT_WIDTH_INCHES, -36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(24 - ROBOT_WIDTH_INCHES, -36))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(specimenHang.end())
                .back(12)
                .build();

        drive.followTrajectorySequence(pushTraj);
        sleep(3000);
        drive.followTrajectorySequence(specimenGrab);

        closeClaw();
        //raise arm
        drive.followTrajectorySequence(specimenHang);

        //slides down
        wrist90();

        drive.followTrajectorySequence(backTraj);

        openClaw();



        return specimenHang.end();
    }
    public Pose2d push2Samples(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        //builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence pushTraj = builder
                //.back(8)
                .splineToConstantHeading(new Vector2d(40,-16), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(47,-16))//left
                /*
                .lineToConstantHeading(new Vector2d(33, -48))//left
                .lineToConstantHeading(new Vector2d(33, -16))//back
                .lineToConstantHeading(new Vector2d(45, -16))//left

                 */
                .lineToConstantHeading(new Vector2d(45, -60))//forward
                .lineToConstantHeading(new Vector2d(45, -16))//back
                .lineToConstantHeading(new Vector2d(55, -16))//left
                .lineToConstantHeading(new Vector2d(55, -60))//forward
                .lineToLinearHeading(new Pose2d(38, -45, Math.toRadians(270)))
                //.lineToConstantHeading(new Vector2d(38, -45))//back
                //.turn(Math.toRadians(270) - startPose.getHeading())
                .build();

        drive.followTrajectorySequence(pushTraj);
        return pushTraj.end();
    }
    public Pose2d hangSpecimen(double fwdDist, double backDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(fwdDist)
                //.lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .back(backDist)
                .build();
        closeClaw();
        arm0();
        wristUp();
        moveArm(0.7, 25, "raiseArm");
        sleep(750);
        wrist90();
        drive.followTrajectorySequenceAsync(forwardTraj);
        drive.waitForIdle();

        moveArm(0.5, 6, "lowerArm");
        sleep(400);

        drive.followTrajectorySequence(backTraj);

        openClaw();
        wristUp();

        return backTraj.end();

    }

    private void reset(){

        armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArm(double speed, int armTarget, String direction){
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
        rightClawS.setPosition(.4);
    }

    public void openClaw() {       //open
        leftClawS.setPosition(.82);
        rightClawS.setPosition(.18);
    }

    //wrist commands

    public void wristDown(){      //down
        wristS.setPosition(0);
    }
    public void wristUp(){        //up
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
        armS.setPosition(0.31);
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

    //slider commands
    /*private void raiseArm(double power){
        frontLeftM.setPower(0);
        frontRightM.setPower(0);
        backLeftM.setPower(0);
        backRightM.setPower(0);
        armM.setPower(-power);
    }
     private void lowerArm(double power){
        frontLeftM.setPower(0);
        frontRightM.setPower(0);
        backLeftM.setPower(0);
        backRightM.setPower(0);
        armM.setPower(power);
    }*/

}