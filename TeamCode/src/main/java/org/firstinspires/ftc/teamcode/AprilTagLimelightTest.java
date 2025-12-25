package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private AprilTagProcessor myAprilTagProcessor;
    private IMU imu;
    private Servo light;
//   List<AprilTagDetection> myAprilTagDetections;
//   AprilTagDetection myAprilTagDetection;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");
        limelight.pipelineSwitch(0);

        light = hardwareMap.get(Servo.class, "shooterLight");
        light.setPosition(0.0);
        //imu = hardwareMap.get(IMU.class, "imu");
        //RevHubOrientationOnRobot revHupOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        //imu.initialize(new IMU.Parameters(revHupOrientationOnRobot));

    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        telemetry.addLine("in loop");


        if (llResult != null && llResult.isValid()){
            //Pose3D botPose = llResult.getBotpose_MT2();
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
            telemetry.addData("Yaw", llResult.getBotpose().getOrientation().getYaw());
            telemetry.addLine("-----------------------");
            telemetry.addData("Tx Main Difference", Math.abs(llResult.getTx()-(-3.25)));
            telemetry.addData("Yaw Main Difference", Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-131.5)));

            telemetry.addData("Tx Back Difference", Math.abs(llResult.getTx()-(0.18)));
            telemetry.addData("Yaw Back Difference", Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-157.2)));

            if (Math.abs(llResult.getTx()-(-3.25)) <=2 && Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-131.5)) <= 5){
                light.setPosition(0.5);
                telemetry.addLine("Light on");
            } else if (Math.abs(llResult.getTx()-(-3.25)) <=2){
                light.setPosition(0.39);
            } else if (Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-131.5)) <= 5){
                light.setPosition(0.61);
            }
            else {
                light.setPosition(0.28);
            }

            if (Math.abs(llResult.getTx()-(0.18)) <=2 && Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-157.2)) <= 5){
                light.setPosition(0.5);
                telemetry.addLine("Light on");
            } else if (Math.abs(llResult.getTx()-(0.18)) <=2){
                light.setPosition(0.39);
            } else if (Math.abs(llResult.getBotpose().getOrientation().getYaw()-(-157.2)) <= 5){
                light.setPosition(0.61);
            }
            else {
                light.setPosition(0.28);
            }
        }





    }
}