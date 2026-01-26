package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotInitializerV2 {
    public static void initializeRobot(HardwareMap hardwareMap, org.firstinspires.ftc.teamcode.RobotHardwareV2 robotHardware){
        double flywheelF = 21.2;
        double flywheelP = 530.0;

        // RobotHardware robotHardware = new RobotHardware();

        robotHardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        robotHardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
        robotHardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        robotHardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        robotHardware.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        robotHardware.intake = hardwareMap.get(DcMotor.class, "intake");

        robotHardware.feeder = hardwareMap.get(Servo.class, "feeder");
        robotHardware.intakeRamp = hardwareMap.get(Servo.class, "intakeRamp");
        robotHardware.rotateLauncher = hardwareMap.get(Servo.class, "launcher");
        robotHardware.gate2 = hardwareMap.get(Servo.class, "gate2");
        robotHardware.gate3 = hardwareMap.get(Servo.class, "gate3");
        robotHardware.spin = hardwareMap.get(CRServo.class, "spin");

        robotHardware.indicatorLightMotor = hardwareMap.get(Servo.class, "motorLight");
        robotHardware.indicatorLightLocation = hardwareMap.get(Servo.class, "locationLight");

        robotHardware.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotHardware.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robotHardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robotHardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        robotHardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        robotHardware.backRight.setDirection(DcMotor.Direction.FORWARD);

        robotHardware.intake.setDirection(DcMotor.Direction.FORWARD);
        robotHardware.flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        robotHardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotHardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotHardware.flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients flywheelPidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF);

        robotHardware.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidfCoefficients);

    }




}
