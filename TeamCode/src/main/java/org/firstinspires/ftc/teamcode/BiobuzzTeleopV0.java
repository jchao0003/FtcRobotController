package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotInitializer;

import java.util.Timer;

@TeleOp
public class BiobuzzTeleopV0 extends OpMode{
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor intake;
    public DcMotorEx flywheel;

    float forward;
    float rotate;
    float strafe;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double maximumMotorPower;
    private Limelight3A limelight;

    BiobuzzRobotHardware robotHardware;

    private ElapsedTime launchTimer = new ElapsedTime();
    final double launchTime = 0.5;


    @Override
    public void init() {

        robotHardware = new BiobuzzRobotHardware();
        BiobuzzRobotInitializer.initializeRobot(hardwareMap, robotHardware);

        backLeft = robotHardware.backLeft;
        backRight = robotHardware.backRight;
        frontLeft = robotHardware.frontLeft;
        frontRight = robotHardware.frontRight;

        intake = robotHardware.intake;
        flywheel = robotHardware.flywheel;

        robotHardware.resetMechanisms();
        robotHardware.intakeRampDown();
    }

    public void loop(){

        robotHardware.setFlywheelVelocity();

        if (gamepad2.leftBumperWasPressed()){
            robotHardware.stopFlywheel();
        }

        if (gamepad2.dpad_down){
            launchTimer.reset();
            robotHardware.setFeedLaunch();
        }

        if (launchTimer.seconds() > launchTime){
            robotHardware.setFeedDown();
        }

        if (gamepad2.aWasPressed()){
            robotHardware.intakeRampDown();
        }

        if (gamepad2.dpad_up){
            robotHardware.setAngleStraight();
        }

        if (gamepad2.right_stick_y > 0.5){
            robotHardware.startIntake();
            robotHardware.startSpin();
        } else if(gamepad2.right_stick_y < -0.5){
            robotHardware.reverseIntake();
            robotHardware.reverseSpin();
        } else {
            robotHardware.stopIntake();
            robotHardware.stopSpin();
        }

        if (Math.abs(robotHardware.getFlywheelVelocityError()) <=50){
            robotHardware.setColorGreenMotor();
        } else {
            robotHardware.setColorRedMotor();
        }



        //MECANUM WHEELS
        //here we are setting our analog values to variables (this is basically what we did to the arm)
        forward = gamepad1.left_stick_y;
        rotate = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        //This is how we end up setting those values
        //You don't really have to memorize something like this just sorta keep in mind how this works
        frontLeftPower = -forward + (strafe + rotate);
        frontRightPower = -forward - (strafe + rotate);
        backLeftPower = -forward - (strafe - rotate);
        backRightPower = -forward + (strafe - rotate);

        //using Java methods you get the maximum power that the motors can reach
        //this is also something you don't really have to worry about understand for FTC coding reasons but it is tapping into more aspects of Java programming outside of robotics
        maximumMotorPower = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));

        //here the power of the motors is set based on the variables
        //these variables aren't completely necessary for movement but they are the best way we've found to have our robot movement
        if (maximumMotorPower > 1) {
            frontLeftPower = frontLeftPower / maximumMotorPower;
            frontRightPower = frontRightPower / maximumMotorPower;
            backLeftPower = backLeftPower / maximumMotorPower;
            backRightPower = backRightPower / maximumMotorPower;


        }

        //ASSIGN POWER MECANUM WHEELS
        if (gamepad1.right_bumper) {                   // slow mode
            frontLeft.setPower(frontLeftPower / 3);
            frontRight.setPower(frontRightPower / 3);
            backLeft.setPower(backLeftPower / 3);
            backRight.setPower(backRightPower / 3);
        } else if (gamepad1.left_bumper) {             // fast mode
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        } else {                                       // comfortable speed
            frontLeft.setPower(frontLeftPower / 2);
            frontRight.setPower(frontRightPower / 2);
            backLeft.setPower(backLeftPower / 2);
            backRight.setPower(backRightPower / 2);
        }



        // Show the elapsed game time and wheel power.
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Flywheel", "error to target (%.0f)", robotHardware.getFlywheelVelocityError());
        telemetry.update();
    }
}
