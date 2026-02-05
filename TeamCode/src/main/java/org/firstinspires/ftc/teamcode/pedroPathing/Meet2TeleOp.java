import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

//@TeleOp(name="Meet2TeleOp", group="Linear Opmode")

public class Meet2TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //classifying our motors
    private DcMotor backLeftM;
    private DcMotor backRightM;
    private DcMotor frontLeftM;
    private DcMotor frontRightM;
    private DcMotor intakeM;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;

    //classifying our servos
    private Servo gateS;
    private Servo standS;

    //classifying the game controller aspects
    private boolean leftBumper;
    private boolean rightBumper;
    private double leftTrigger;
    private double rightTrigger;
    private double gamepad2Y;

    //servo set position variables
    private double gateSPosition = 0.7;
    private double standSPosition = 0.0;

    private DcMotor spin;

    //VARIABLES
    float forward;
    float rotate;
    float strafe;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    double flywheelPower;

    double maximumMotorPower;
    double yInputtedPower;
    double yInputtedPower2;


    @Override
    public void runOpMode() { //this is the big important loop that all your code runs on

        frontRightM = hardwareMap.get(DcMotor.class, "frontRightM");
        backRightM = hardwareMap.get(DcMotor.class, "backRightM");
        frontLeftM = hardwareMap.get(DcMotor.class, "frontLeftM");
        backLeftM = hardwareMap.get(DcMotor.class, "backLeftM");
        intakeM = hardwareMap.get(DcMotor.class, "intakeM");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        gateS = hardwareMap.get(Servo.class, "gateS");
        standS = hardwareMap.get(Servo.class, "standS");

        frontRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftM.setDirection(DcMotor.Direction.FORWARD);
        frontRightM.setDirection(DcMotor.Direction.REVERSE);
        backLeftM.setDirection(DcMotor.Direction.FORWARD);
        backRightM.setDirection(DcMotor.Direction.REVERSE);
        intakeM.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        frontLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
        standS.setPosition(0.075);


        while (opModeIsActive()) { //this is the other important loop that actually makes your robot move!
            // runs until the end of the match (driver presses STOP)

            //Setting the joysticks on the 2nd game controller to control the arm movement (this could also be done with encoders it is really based on driver's preference)
            yInputtedPower = gamepad2.left_stick_y;
            yInputtedPower2 = gamepad2.right_stick_y;

        /*we used set values to ensure the arm moves at a consistent
        speed but you could also set it to the joystick value using
        armM.setPower(yInputtedPower); or armM.setPower(gamepad2.left_stick_y); */

            //servo set positions

            gateS.setPosition(0.8);

            if (gamepad2.dpad_down){
                gateSPosition = 1;
                gateS.setPosition(gateSPosition);
                sleep(500);
                gateSPosition = 0.8;
                gateS.setPosition(gateSPosition);
            }

            if (gamepad2.dpad_up){
                standS.setPosition(standSPosition);
            }

            if (gamepad2.right_bumper){
                flywheel.setVelocity(1475);
                flywheel2.setVelocity(1475);
            } else {
                flywheel.setVelocity(1050);
                flywheel2.setVelocity(1050);
            }



            if (gamepad1.y){
                frontLeftM.setDirection(DcMotor.Direction.REVERSE);
                frontRightM.setDirection(DcMotor.Direction.FORWARD);
                backLeftM.setDirection(DcMotor.Direction.REVERSE);
                backRightM.setDirection(DcMotor.Direction.FORWARD);

                forward = gamepad1.left_stick_y;
                rotate = gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_y;

                frontLeftPower = forward - (strafe + rotate);
                frontRightPower = forward + (strafe + rotate);
                backLeftPower = forward + (strafe - rotate);
                backRightPower = forward - (strafe - rotate);

            } else {
                frontLeftM.setDirection(DcMotor.Direction.FORWARD);
                frontRightM.setDirection(DcMotor.Direction.REVERSE);
                backLeftM.setDirection(DcMotor.Direction.FORWARD);
                backRightM.setDirection(DcMotor.Direction.REVERSE);

                frontLeftPower = forward - (strafe + rotate);
                frontRightPower = forward + (strafe + rotate);
                backLeftPower = forward + (strafe - rotate);
                backRightPower = forward - (strafe - rotate);
            }



//code practice
            //MECANUM WHEELS
            //here we are setting our analog values to variables (this is basically what we did to the arm)
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
            //This is how we end up setting those values
            //You don't really have to memorize something like this just sorta keep in mind how this works
            frontLeftPower = forward - (strafe + rotate);
            frontRightPower = forward + (strafe + rotate);
            backLeftPower = forward + (strafe - rotate);
            backRightPower = forward - (strafe - rotate);


            flywheelPower = (flywheel.getVelocity() + flywheel2.getVelocity())/2;

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
                frontLeftM.setPower(frontLeftPower / 3);
                frontRightM.setPower(frontRightPower / 3);
                backLeftM.setPower(backLeftPower / 3);
                backRightM.setPower(backRightPower / 3);
            } else if (gamepad1.left_bumper) {             // comfortable speed
                frontLeftM.setPower(frontLeftPower);
                frontRightM.setPower(frontRightPower);
                backLeftM.setPower(backLeftPower);
                backRightM.setPower(backRightPower);
            } else {                                       // fast mode
                frontLeftM.setPower(frontLeftPower / 2);
                frontRightM.setPower(frontRightPower / 2);
                backLeftM.setPower(backLeftPower / 2);
                backRightM.setPower(backRightPower / 2);
            }

            //ASSIGN POWER LAUNCHER/INTAKE
            // if (gamepad2.y){
            //   flywheel.setPower(0.87);
            // } else
            if (gamepad2.a){
                intakeM.setPower(0.9);
            } else if (gamepad2.y){
                intakeM.setPower(-0.9);
            } else {
                // flywheel.setPower(0);
                intakeM.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Flywheel", "motor be like: weeeee (%.2f)", flywheelPower);
            telemetry.update();
        }
    }




  /* This is the type of method you would make if you wanted to set motor movement to a digital input

    private void armMovement(int distance, int time, double power) {
      armM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      armM.setTargetPosition(distance);
      armM.setPower(power);

      armM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      sleep(time);
    }

  This method would then be used with something like this:
    if (gamepad2.a){
      armMovement(100, 1000, 0.5);
    } else if (gamepad2.x){
    ar  mMovement(200, 1000, 0.6);
    }

  Choices like these are dependent on communication with drivers and their comfortability
  */



} //The end of the program