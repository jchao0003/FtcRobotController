package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColorSensorsInit {

    NormalizedColorSensor gate2sensor;
    NormalizedColorSensor gate3sensor;


    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap) {
        gate2sensor = hardwareMap.get(NormalizedColorSensor.class, "gate2sensor");
        gate3sensor = hardwareMap.get(NormalizedColorSensor.class, "gate3sensor");

        gate2sensor.setGain(10);
        gate3sensor.setGain(10);
    }

    public DetectedColor getDetectedColor2(Telemetry telemetry) {
        NormalizedRGBA color2 = gate2sensor.getNormalizedColors();

        float normRed2, normGreen2, normBlue2;
        normRed2 = color2.red / color2.alpha;
        normGreen2 = color2.green / color2.alpha;
        normBlue2 = color2.blue / color2.alpha;

        telemetry.addData("Red 2: ", normRed2);
        telemetry.addData("Green 2: ", normGreen2);
        telemetry.addData("Blue 2: ", normBlue2);

        if ((normRed2 > 0.36 && normGreen2 > 0.43 && normBlue2 > 0.72)) {
            return DetectedColor.PURPLE;
        } else if (normRed2 > 0.15 && normGreen2 > 0.63 && normBlue2 > 0.47) {
            return DetectedColor.GREEN;
        } else if ((normRed2 > 0.16 && normGreen2 > 0.26 && normBlue2 > 0.17)){
            return DetectedColor.GREEN;
        } else if ((normRed2 > 0.14 && normGreen2 > 0.17 && normBlue2 > 0.13)){
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
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

    }

    public DetectedColor getDetectedColor3(Telemetry telemetry) {
        NormalizedRGBA color3 = gate3sensor.getNormalizedColors();

        float normRed3, normGreen3, normBlue3;
        normRed3 = color3.red / color3.alpha;
        normGreen3 = color3.green / color3.alpha;
        normBlue3 = color3.blue / color3.alpha;

        telemetry.addData("Red 3: ", normRed3);
        telemetry.addData("Green 3: ", normGreen3);
        telemetry.addData("Blue 3: ", normBlue3);

        if ((normRed3 > 0.36 && normGreen3 > 0.43 && normBlue3 > 0.72)) {
            return DetectedColor.PURPLE;
        } else if (normRed3 > 0.15 && normGreen3 > 0.63 && normBlue3 > 0.47) {
            return DetectedColor.GREEN;
        } else if ((normRed3 > 0.13 && normGreen3 > 0.26 && normBlue3 > 0.17)){
            return DetectedColor.GREEN;
        } else if ((normRed3 > 0.12 && normGreen3 > 0.21 && normBlue3 > 0.20)){
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }

        /* PURPLE
            red:
            green:
            blue:
           GREEN
            red:
            green:
            blue:
           PURPLE HOLE
            red:
            green:
            blue:
           GREEN HOLE
            red:
            green:
            blue:
         */
    }
}


