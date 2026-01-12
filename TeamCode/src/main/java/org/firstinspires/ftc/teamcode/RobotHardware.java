package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor intake;
    public DcMotorEx flywheel;

    public Servo feeder;
    public Servo intakeRamp;
    public Servo rotateLauncher;
    public Servo indicatorLight;

    public double backVelocity = 1620;
    public double frontVelocity = 1100;
    public double middleVelocity = 1200;

    private double flywheelTargetVelocity = 0;

    public void launch(){
        feeder.setPosition(.77);
        sleep(250);
        feeder.setPosition(0.47);
        sleep(100);
    }

    public void startIntake(){
        intake.setPower(.5);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
    public void resetMechanisms(){
        feeder.setPosition(0.47);
        intakeRamp.setPosition(0.465);
        rotateLauncher.setPosition(0.2);
        sleep(100);
    }

    public void reverseIntake(){
        intake.setPower(-0.5);
    }

    public void stopFlywheel(){
        flywheel.setPower(0);
    }

    public void setFlywheelSpeedBackPosition(){
        flywheel.setVelocity(backVelocity);
        flywheelTargetVelocity = backVelocity;
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

    public void lightOff(){
        indicatorLight.setPosition(0.0);
    }
    public void setColorGreen(){
        indicatorLight.setPosition(0.5);
    }

    public void setColorRed(){
        indicatorLight.setPosition(0.28);
    }

    public void setBlueAngle(){
        rotateLauncher.setPosition(0.222);
    }
}
