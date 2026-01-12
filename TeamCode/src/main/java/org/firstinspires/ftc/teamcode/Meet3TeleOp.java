package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotInitializer;

@TeleOp
public class Meet3TeleOp extends OpMode{
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

    RobotHardware robotHardware;

    @Override
    public void init() {

        robotHardware = new RobotHardware();
        RobotInitializer.initializeRobot(hardwareMap, robotHardware);

        backLeft = robotHardware.backLeft;
        backRight = robotHardware.backRight;
        frontLeft = robotHardware.frontLeft;
        frontRight = robotHardware.frontRight;

        intake = robotHardware.intake;
        flywheel = robotHardware.flywheel;

        robotHardware.resetMechanisms();
    }

    public void loop(){

        if (gamepad2.dpad_down){
            robotHardware.launch();
        }

        if (gamepad2.right_bumper){
            robotHardware.setFlywheelSpeedBackPosition();
        } else if (gamepad2.dpad_left){
            robotHardware.stopFlywheel();
            robotHardware.lightOff();
        } else if (gamepad2.left_bumper) {
            robotHardware.setFlywheelSpeedMiddlePosition();
        } else {
            robotHardware.setFlywheelSpeedFrontPosition();
        }

        if (gamepad2.xWasPressed()){
            robotHardware.setBlueAngle();
        }

        if (gamepad2.a){
            robotHardware.startIntake();
        } else if (gamepad2.y){
            robotHardware.reverseIntake();
        } else {
            robotHardware.stopIntake();
        }

        if (gamepad2.dpad_left){
            robotHardware.intakeRampUp();
        }

        if (Math.abs(robotHardware.getFlywheelVelocityError()) <=50){
            robotHardware.setColorGreen();
        } else {
            robotHardware.setColorRed();
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
