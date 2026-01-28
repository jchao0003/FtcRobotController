package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareV2 {
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor intake;
    public DcMotorEx flywheel;

    public Servo feeder;
    public Servo intakeRamp;
    public Servo rotateLauncher;
    public Servo indicatorLightMotor;
    public Servo indicatorLightLocation;
    public Servo gate2;
    public Servo gate3;
    public CRServo spin;

    public double backVelocity = 800; // 1600;
    public double frontVelocity = 1100;
    public double middleVelocity = 1200;

    private double flywheelTargetVelocity = 0;
    final double TIME_BETWEEN_SHOTS = 1.1;
    final double FEED_TIME = 0.3;

    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();

    private LaunchState launchState = LaunchState.IDLE;

//
//    public void launch(){
//        feeder.setPosition(.77);
//        sleep(200);
//        feeder.setPosition(0.47);
//    }

    public void dropGate2(){
        gate2.setPosition(0.84);
    }

    public void dropGate3(){
        gate3.setPosition(0.87);
    }

    private enum LaunchState {
        IDLE,
        START_LAUNCH,
        WAIT_LAUNCH_COMPLETE,
    }
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.START_LAUNCH;
                    shotTimer.reset();
                }
                break;
            case START_LAUNCH:
                if (flywheel.getVelocity() > flywheelTargetVelocity - 60){
                    launchState = LaunchState.WAIT_LAUNCH_COMPLETE;
                    stopSpin();
                    feeder.setPosition(0.40);


                    feederTimer.reset();
                }
                break;
            case WAIT_LAUNCH_COMPLETE:
                if (feederTimer.seconds() > FEED_TIME) {
                    feeder.setPosition(0.66);
                    startSpin();

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    public void manualLaunch(){
        feeder.setPosition(0.40);
    }

    public void manualReset(){
        feeder.setPosition(0.66);
    }

    public void startSpin(){
        spin.setPower(-1);
    }

    public void stopSpin(){
        spin.setPower(0);
    }


    public void launchPt2(){
        //feeder.setPosition(0.68);
    }

    public void startIntake(){
        intake.setPower(.7);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
    public void resetMechanisms(){
        feeder.setPosition(0.66);
        intakeRamp.setPosition(0.4655);
        rotateLauncher.setPosition(0.2);
        gate2.setPosition(0.5);
        gate3.setPosition(0);
    }

    public void reverseIntake(){
        intake.setPower(-0.7);
    }

    public void stopFlywheel(){
        flywheel.setVelocity(0.0);
    }

    public void setFlywheelSpeedBackPosition(){
        flywheel.setVelocity(backVelocity);
        flywheelTargetVelocity = backVelocity;
    }

    public void intakeRampUp(){
        intakeRamp.setPosition(0.5185);
    }

    public void intakeRampDown(){
        intakeRamp.setPosition(0.462);
    }

    public void setFlywheelSpeedFrontPosition(){
        flywheel.setVelocity(frontVelocity);
        flywheelTargetVelocity = frontVelocity;
    }

    public void setFlywheelSpeedMiddlePosition(){
        flywheel.setVelocity(middleVelocity);
        flywheelTargetVelocity = middleVelocity;
    }

    public double getFlywheelVelocityError() {
        return flywheelTargetVelocity - flywheel.getVelocity();
    }

    public void motorLightOff(){
        indicatorLightMotor.setPosition(0.0);
    }
    public void setColorGreenMotor(){
        indicatorLightMotor.setPosition(0.5);
    }

    public void setColorRedMotor(){
        indicatorLightMotor.setPosition(0.277);
    }

    public void locationLightOff(){
        indicatorLightLocation.setPosition(0.0);
    }

    public void setColorBlueLocation(){
        indicatorLightLocation.setPosition(0.611);
    }

    public void setColorOrangeLocation(){
        indicatorLightLocation.setPosition(0.388);
    }

    public void setBlueAngle(){
        rotateLauncher.setPosition(0.222);
    }

    public void setRedAngle(){
        rotateLauncher.setPosition(0.16);
    }

    public void setAngleStraight(){
        rotateLauncher.setPosition(0.195);
    }
}
