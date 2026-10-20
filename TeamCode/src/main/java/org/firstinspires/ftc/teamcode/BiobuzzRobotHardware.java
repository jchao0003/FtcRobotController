package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class BiobuzzRobotHardware {
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor intake;
    public DcMotorEx flywheel;

    public Servo feeder;
    public Servo intakeRamp;
    public Servo rotateLauncher;
    public Servo trajectoryServo;
    public Servo indicatorLightMotor;
    public Servo indicatorLightLocation;
    public Servo gate2;
    public Servo gate3;
    public CRServo spin;

    public Limelight3A limelight;

    public double velocity = 1100;

    private double flywheelTargetVelocity = 0;

    public void setFeedDown(){
        feeder.setPosition(0.66);
    }

    public void setFeedLaunch(){
        feeder.setPosition(0.30);
    }

    public void startSpin(){
        spin.setPower(-1);
    }

    public void reverseSpin(){
        spin.setPower(1);
    }

    public void stopSpin(){
        spin.setPower(0);
    }

    public void startIntake(){
        intake.setPower(.70);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void resetMechanisms(){
        setFeedDown();
        intakeRampDown();
        setAngleStraight();
    }

    public void reverseIntake(){
        intake.setPower(-0.7);
    }

    public void stopFlywheel(){
        flywheel.setVelocity(0.0);
    }

    public void setFlywheelVelocity(){
        flywheel.setVelocity(velocity);
    }

    public void intakeRampDown(){
        intakeRamp.setPosition(0.462);
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

    public void setAngleStraight(){
        rotateLauncher.setPosition(0.2); //0.195); //0.031/18
    }
}
