package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.arcrobotics.ftclib.controller.PDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@Configurable
public class colorSensors {

    private NormalizedColorSensor leftSensor;
    private NormalizedColorSensor rightSensor;

    public double flywheelVelocity = 0;

    public static double redThresh=0;
    public static double blueThresh=0;
    public static double greenThresh=0;


    public colorSensors(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");

    }
// -1=nothing detected
// 0 =purple
// 1 =green
    public int getColorLeft() {
        NormalizedRGBA colors = leftSensor.getNormalizedColors();
        if ((colors.red + colors.blue) / 2 > colors.green) {
            if (colors.red > redThresh && colors.blue > blueThresh) {
                return 0;
            } else {
                return -1;
            }
        } else {
            if (colors.green>greenThresh) {
                return 1;
            } else {
                return -1;
            }
        }
    }
    public int getColorRight() {
        NormalizedRGBA colors = rightSensor.getNormalizedColors();
        if ((colors.red + colors.blue) / 2 > colors.green) {
            if (colors.red > redThresh && colors.blue > blueThresh) {
                return 0;
            } else {
                return -1;
            }
        } else {
            if (colors.green>greenThresh) {
                return 1;
            } else {
                return -1;
            }
        }
    }






}
