package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name="FlywheelTesting", group="Linear Opmode")


public class FlywheelTesting extends LinearOpMode{

    // todo: write your code here

    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    private Servo gateS;

    private double gateSPosition = 0.8;

    private double flywheelVelocity = 800.0;

    double resultMaxVelocityTest = 1900;
    double F = 32767.0/resultMaxVelocityTest;
    double kP = F * 0.1;
    double kI = kP * 0.1;
    double kD = kI * 0.1;
    double position = 5.0;

    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        gateS = hardwareMap.get(Servo.class, "gateS");

        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);

        flywheel.setVelocityPIDFCoefficients(kP,kI,kD,F);
        flywheel2.setVelocityPIDFCoefficients(kP,kI,kD,F);

        waitForStart();
        while (opModeIsActive()){

            gateS.setPosition(0.8);

            flywheel.setVelocity(flywheelVelocity);
            flywheel2.setVelocity(flywheelVelocity);
            telemetry.addData("Velocity:", (flywheel.getVelocity() + flywheel2.getVelocity())/2);
            telemetry.update();

            if (gamepad2.dpad_down){
                gateSPosition = 1;
                gateS.setPosition(gateSPosition);
                sleep(500);
                gateSPosition = 0.8;
                gateS.setPosition(gateSPosition);
                sleep(250);
            }
        }


    }



}


