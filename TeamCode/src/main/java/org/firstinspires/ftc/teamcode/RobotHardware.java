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
    public Servo indicatorLightMotor;
    public Servo indicatorLightLocation;

    public double backVelocity = 1620;
    public double frontVelocity = 1100;
    public double middleVelocity = 1200;

    private double flywheelTargetVelocity = 0;

    public void launch(){
        feeder.setPosition(.77);
        sleep(200);
        feeder.setPosition(0.47);
        sleep(1000);
        feeder.setPosition(0.62);
    }

    public void startIntake(){
        intake.setPower(.5);
    }

    public void stopIntake(){
        intake.setPower(0);
    }
    public void resetMechanisms(){
        feeder.setPosition(0.47);
        intakeRamp.setPosition(0.462);
        rotateLauncher.setPosition(0.2);
        sleep(100);
    }

    public void reverseIntake(){
        intake.setPower(-0.5);
    }

    public void stopFlywheel(){
        flywheel.setVelocity(0.0);
    }

    public void setFlywheelSpeedBackPosition(){
        flywheel.setVelocity(backVelocity);
        flywheelTargetVelocity = backVelocity;
    }

    public void intakeRampUp(){
        intakeRamp.setPosition(0.51);
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
}
