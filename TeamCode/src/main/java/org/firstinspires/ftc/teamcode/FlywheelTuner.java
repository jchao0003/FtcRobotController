package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FlywheelTuner extends OpMode {
    public DcMotorEx flywheel1;
    //public DcMotorEx flywheel2;

    public double highVelocity = 1620; //middle distance (1620 V2) 410
    public double lowVelocity = 1100; //1300

    double curTargetVelocity = highVelocity;

    double F = 0.0; //18.046(V1) 19.45(V2) 21.2
    double P = 0.0; //445(V1) 250(V2) 530

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 0;

    double highestError;

    boolean hasReachedTarget = false;



    @Override
    public void init(){
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel");
        //flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        //flywheel2.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop(){
        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if (gamepad1.leftBumperWasPressed()){
            curTargetVelocity -= stepSizes[stepIndex];
        }

        if (gamepad1.rightBumperWasPressed()){
            curTargetVelocity += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel1.setVelocity(curTargetVelocity);
        //flywheel2.setVelocity(curTargetVelocity);

        //double curVelocity = (flywheel1.getVelocity() + flywheel2.getVelocity())/2;
        double curVelocity = flywheel1.getVelocity();
        double error = curTargetVelocity - curVelocity;

        if (curVelocity == curTargetVelocity){
            hasReachedTarget = true;
        }

        if (hasReachedTarget){
            if (error > highestError){
                highestError = error;
            }
        }

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Highest Error", "%.2f", highestError);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
