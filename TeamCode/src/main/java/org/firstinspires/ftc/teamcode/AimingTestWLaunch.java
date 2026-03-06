package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp

public class AimingTestWLaunch extends OpMode{

    public DcMotorEx flywheel;

    public Servo feeder;
    public Servo rotateLauncher;

    RobotHardwareV2 robotHardware;

    public Limelight3A limelight;

    @Override
    public void init(){

        robotHardware = new RobotHardwareV2();
        RobotInitializerV2.initializeRobot(hardwareMap, robotHardware);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        feeder = hardwareMap.get(Servo.class, "feeder");
        rotateLauncher = hardwareMap.get(Servo.class, "launcher");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");
        limelight.pipelineSwitch(0);

        rotateLauncher.setPosition(0.195);
    }

    public void start(){
        limelight.start();

    }

    public void loop(){
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0;
        int id = 0;

        telemetry.addData("Tx: ", Tx);

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20 ||fiducial.getFiducialId() == 24){
                id = fiducial.getFiducialId(); // The ID number of the fiducial
                Tx = fiducial.getTargetXDegrees();
            }
        }

        telemetry.addData("April Tag Detected is: ", id);
        telemetry.update();


        if (llResult != null && llResult.isValid() && id == 20) {

            if (Tx >= 0) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - (0.002*Math.abs(Tx)));
                sleep(500);
            } else if (Tx <= -3){
                rotateLauncher.setPosition(rotateLauncher.getPosition() + (0.002*Math.abs(Tx+3)));
                sleep(500);
            }
        }

        if (llResult != null && llResult.isValid() && id == 24) {

            if (Tx >= 5) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - (0.002*Math.abs(Tx-5)));
                sleep(500);
            } else if (Tx <= -1) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() + (0.002*Math.abs(Tx+1)));
                sleep(500);
            }
        }



        if (gamepad2.right_bumper){
            robotHardware.setFlywheelSpeedMiddlePosition();
        } else if (gamepad2.left_bumper) {
            robotHardware.setFlywheelSpeedBackPosition();
            robotHardware.setFarTrajectory();
        } else if (gamepad2.y){
            robotHardware.setFlywheelSpeedFrontPosition();
        } else {
            robotHardware.stopFlywheel();
            robotHardware.setNormalTrajectory();
        }

        if (gamepad2.dpad_down){
            robotHardware.setFeedLaunch();
        }

        if (gamepad2.a){
            robotHardware.setFeedDown();
        }


    }


    // todo: write your code here

    public boolean adjustLauncherUsingAprilTagRedBack(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            if (Tx >= 0) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                return false;
            } else if (Tx <= -3){
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagBlueBack(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

             if (Tx >= 5) {
                 rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                 return false;
             } else if (Tx <= -1) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagRedFront(Telemetry telemetry){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            telemetry.addData("Tx: ", Tx);
            telemetry.addData("launchPos: ", rotateLauncher.getPosition());
            if (Tx >= 4) {
                double newPosition = rotateLauncher.getPosition() - 0.002;
                rotateLauncher.setPosition(newPosition);
                telemetry.addData("newPosition minus", newPosition);
                return false;
            } else if (Tx <= -6){
                double newPosition = rotateLauncher.getPosition() + 0.002;
                rotateLauncher.setPosition(newPosition);
                telemetry.addData("newPosition plus", newPosition);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }

    public boolean adjustLauncherUsingAprilTagBlueFront(){
        return true;
        /*
        LLResult llResult = limelight.getLatestResult();
        double Tx = 0.0;

        if (llResult != null && llResult.isValid()) {
            Tx = llResult.getTx();

            if (Tx >= 5) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() - 0.002);
                return false;
            } else if (Tx <= -5) {
                rotateLauncher.setPosition(rotateLauncher.getPosition() + 0.002);
                return false;
            } else {
                return true;
            }
        }
        return false;
         */
    }
}