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

        double START_POSITION_X = 1 + (ROBOT_WIDTH_INCHES / 2);
        double START_POSITION_Y = -72 + (ROBOT_LENGTH_INCHES/2);
        // start position of red side, right of center
        Pose2d startPose = new Pose2d(START_POSITION_X, START_POSITION_Y, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        //auto code goes here

        //set arm, wrist, and claw
        /*reAdjust
        closeClaw();
        sleep(1000);
        wristMid();
        sleep(500);
        armDown();
        sleep(1000);

         */

        //hang preset specimen on high rung
        /* hangSpecimen
        moveArm(0.1, 10, "raiseArm");
        wristMid();
        sleep(2150);
        drive(0.1, 25, 25, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.4, 5, "lowerArm");
        sleep(1000);
        drive(0.2, -5, -5, "straight");
        openClaw();
        sleep(1000);


         */
        Pose2d latestPose = startPose;
        latestPose = hangSpecimen(36 - ROBOT_LENGTH_INCHES/2, 4, drive, latestPose);
        //latestPose = push2Samples(drive, latestPose);
        latestPose = push1Sample(drive, latestPose);



        //set slides, arm, wrist, and claw
        //wristUp();
        sleep(500);
        //armDown();
        sleep(1000);


     /*
        wristMid();
        sleep(500);
        closeClaw();
        sleep(500);*/


        //pick up new specimen
        /*drive(0.2, 17, 17, "turnRight");
        drive(0.0, 0, 0, "straight");
        drive(0.2, 10, 10, "right");
        drive(0.0, 0, 0, "straight");
        drive(0.2, 10, 10, "straight");
        drive(0.0, 0, 0, "straight");
        closeClaw();
        sleep(500);

        hang new specimen
        drive(0.3, -48, -48, "straight");
        drive(0.2, 17, 16, "turnLeft");
        wristMid();
        sleep(500);
        moveArm(0.2, 10, "raiseArm");
        sleep(2200);
        drive(0.2, 16, 16, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.4, 3, "lowerArm");
        openClaw();
        sleep(1000);
        drive(0.3, -12, -12, "straight");

        //reset slides, arm, wrist, and claw
        moveArm(0.3, 10, "lowerArm");
        sleep(2300);
        closeClaw();
        sleep(1000);
        wristUp();
        sleep(500);
        armDown();
        sleep(1000);

        //push sample to observation zone
        drive(0.3, 24, 24, "right");
        drive(0.3, 24, 24, "straight");
        drive(0.2, 5, 5, "right");
        drive(0.3, -48, -48, "straight");
        drive(0.3, 10, 10, "straight");

        //pick up new specimen
        drive(0.2, 16, 16, "turnRight");
        drive(0.2, 10, 10, "right");
        drive(0.2, 10, 10, "straight");
        closeClaw();
        sleep(500);

        //hang new specimen
        drive(0.3, -48, -48, "straight");
        drive(0.2, 16, 16, "turnLeft");
        wristMid();
        sleep(500);
        moveArm(0.2, 10, "raiseArm");
        sleep(2200);
        drive(0.2, 16, 16, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.4, 3, "lowerArm");
        openClaw();
        sleep(1000);
        drive(0.3, -24, -24, "straight");

        drive(0.3, 56, 56, "right");*/






    }
    public Pose2d push1Sample(SampleMecanumDrive drive, Pose2d startPose) {

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence pushTraj = builder
                //.back(8)
                .turn(Math.toRadians(270) - startPose.getHeading())
                .strafeLeft(24)
                .back(24)
                .strafeLeft(9)
                .forward(46)
                .back(14)
                .build();
        TrajectorySequence specimenGrab = drive.trajectorySequenceBuilder(pushTraj.end())
                .lineTo(new Vector2d(38,-71 + ROBOT_LENGTH_INCHES/2))
                .build();
        TrajectorySequence specimenHang = drive.trajectorySequenceBuilder(specimenGrab.end())
                .back(6)
                .turn(Math.toRadians(90) - specimenGrab.end().getHeading())
               // .splineToLinearHeading(new Pose2d(24 - ROBOT_WIDTH_INCHES, -36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(24 - ROBOT_WIDTH_INCHES, -36))
                .build();




        drive.followTrajectorySequence(pushTraj);
        sleep(3000);
        drive.followTrajectorySequence(specimenGrab);
        closeClaw();
        drive.followTrajectorySequence(specimenHang);

        return specimenHang.end();
    }
    public Pose2d push2Samples(SampleMecanumDrive drive, Pose2d startPose) {

    /*
    drive(0.0, 0, 0, "straight");
    drive(0.2, -7, -7, "straight");
    drive(0.1, 48, 48, "turnRight");

     */
        /*
        turn to heading 270
        strafe left 36 in
        backwards 24 in
        strafe left 12 in
        forward 54 in
        backwards 54 in
        strafe left 10 in
        forward 54 in

         */
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence pushTraj = builder
                //.back(8)
                .turn(Math.toRadians(270) - startPose.getHeading())
                .strafeLeft(24)
                .back(24)
                .strafeLeft(9)
                .forward(46)
                .back(46)
                .strafeLeft(9)
                .forward(46)
                .build();
        drive.followTrajectorySequence(pushTraj);
        return pushTraj.end();
    }
    public Pose2d hangSpecimen(double fwdDist, double backDist, SampleMecanumDrive drive, Pose2d startPose){
        /*
        moveArm(0.1, 8, "raiseArm");
        wristMid();
        sleep(2150);
        drive(0.3, distance, distance, "straight");
        drive(0.1, 1, 1, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.4, 4, "lowerArm");
        sleep(1500);
        drive(0.3, -4, -4, "straight");
        drive(0.0, 0, 0, "straight");

         */

        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(fwdDist)
                //.lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .back(backDist)
                .build();


        moveArm(0.1, 8, "raiseArm");
        wristMid();

        sleep(2150);
        drive.followTrajectorySequence(forwardTraj);

        moveArm(0.4, 4, "lowerArm");

        sleep(1500);
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

    public void closeClaw(){      //close
        leftClawS.setPosition(.57);
        rightClawS.setPosition(.37);
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
    public void wristMid(){       //halfway
        wristS.setPosition(0.7);
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

}