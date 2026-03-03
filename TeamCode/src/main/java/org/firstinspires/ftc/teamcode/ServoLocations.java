package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoLocations extends OpMode {

    public Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "trajectoryServo");
        servo.setPosition(0.5);
    }

    @Override
    public void loop() {

    }
}
