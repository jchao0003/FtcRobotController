package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColorSensorsInit {

    NormalizedColorSensor gate2sensor;
    NormalizedColorSensor gate3sensor;


    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap){
        gate2sensor = hardwareMap.get(NormalizedColorSensor.class, "gate3sensor");
        gate3sensor = hardwareMap.get(NormalizedColorSensor.class, "gate2sensor");

        gate2sensor.setGain(10);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA color2 = gate2sensor.getNormalizedColors();
        NormalizedRGBA color3 = gate3sensor.getNormalizedColors();

        float normRed2, normGreen2, normBlue2;
        normRed2 = color2.red / color2.alpha;
        normGreen2 = color2.green / color2.alpha;
        normBlue2 = color2.blue / color2.alpha;

        telemetry.addData("Red: ", normRed2);
        telemetry.addData("Green: ", normGreen2);
        telemetry.addData("Blue: ", normBlue2);

        if((normRed2>0.36 && normGreen2>0.43 && normBlue2>0.72)){
            return DetectedColor.PURPLE;
        } else if (normRed2>0.15 && normGreen2>0.63 && normBlue2>0.47){
            return DetectedColor.GREEN;
        } else {
            return DetectedColor.UNKNOWN;
        }

    }
}

/* PURPLE
    red: >0.36
    green: >0.43
    blue: >0.72
   GREEN
    red: >0.15
    green: >0.63
    blue: >0.47
   PURPLE HOLE
    red: >0.14
    green: >0.17
    blue: >0.13
   GREEN HOLE
    red: >0.13
    green: >0.26
    blue: >0.17
 */
