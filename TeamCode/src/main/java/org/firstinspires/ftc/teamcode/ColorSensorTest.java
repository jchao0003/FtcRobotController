package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class ColorSensorTest extends OpMode {
    TestColorSensorsInit sensorInit = new TestColorSensorsInit();

    TestColorSensorsInit.DetectedColor detectedColor2;

    public void init(){
        sensorInit.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor2 = sensorInit.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor2);
    }
}
