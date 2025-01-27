package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.util.Hardware;

import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name="RightAutoM3DW", group="Linear Opmode")

public class RightAutoM3DW extends LinearOpMode {

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

        double START_POSITION_X =  (ROBOT_WIDTH_INCHES / 2);
        double START_POSITION_Y = -72 + (ROBOT_LENGTH_INCHES / 2);
        // start position of red side, right of center
        Pose2d startPose = new Pose2d(START_POSITION_X, START_POSITION_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d latestPose = startPose;
        latestPose = hangSpecimen(28 - ROBOT_LENGTH_INCHES / 2, 9, drive, latestPose);
        latestPose = push2Samples(drive, latestPose);
        latestPose = hangGrabSpecimen(drive, latestPose);
        //latestPose = push1Sample(drive, latestPose);
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
                .back(10)
                .build();

        drive.followTrajectorySequence(specimenGrab);

        closeClaw();
        //raise arm
        sleep(500);

        drive.followTrajectorySequenceAsync(specimenHang);
        arm0();
        wristUp();
        moveArm(0.7, 25, "raiseArm");
        sleep(750);
        wrist90();

        drive.waitForIdle();

        moveArm(0.5, 5, "lowerArm");
        sleep(400);

        drive.followTrajectorySequence(backTraj);

        openClaw();



        return specimenHang.end();
    }


    public Pose2d push1Sample(SampleMecanumDrive drive, Pose2d startPose) {

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence pushTraj = builder
                //.back(8)
                .turn(Math.toRadians(270+10) - startPose.getHeading())
                .lineToConstantHeading(new Vector2d(33, -48))//left
                .lineToConstantHeading(new Vector2d(33, -16))//back
                .lineToConstantHeading(new Vector2d(45, -20))//left
                .lineToConstantHeading(new Vector2d(45, -60))//forward
                .lineToConstantHeading(new Vector2d(38, -48))//back
                .build();
        TrajectorySequence specimenGrab = drive.trajectorySequenceBuilder(pushTraj.end())
                .lineTo(new Vector2d(48,-70 + ROBOT_LENGTH_INCHES/2))
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
        drive.followTrajectorySequenceAsync(specimenHang);
        sleep(500);
        moveArm(0.7, 26, "raiseArm");
        drive.waitForIdle();
        wrist90();
        moveArm(0.5, 5, "low");


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

        drive.followTrajectorySequenceAsync(pushTraj);
        moveArm(0.7, 30, "lowerArm");
        drive.waitForIdle();
        return pushTraj.end();
    }
    public Pose2d hangSpecimen(double fwdDist, double backDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(fwdDist)
                //.lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .lineToConstantHeading(new Vector2d(22, -48))
                .build();
        closeClaw();
        arm0();
        wristUp();
        moveArm(0.7, 26, "raiseArm");
        sleep(750);
        wrist90();
        drive.followTrajectorySequenceAsync(forwardTraj);
        drive.waitForIdle();

        moveArm(0.5, 5, "lowerArm");
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
        rightClawS.setPosition(.33);
    }

    public void openClaw(){       //open
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

    public void armUp(){          //up
        armS.setPosition(0.115);
    }
    public void armDown(){        //down
        armS.setPosition(0);
    }
    public void armMid(){         //halfway
        armS.setPosition(0.4);
    }
    public void armLow(){         //-45 degrees
        armS.setPosition(0.6);
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
    public void arm0() {        //all the way down
        armS.setPosition(0);
    }
}
