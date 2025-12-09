package org.firstinspires.ftc.teamcode.V2.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.powerable.SetPower;

public class ColorSensors implements Subsystem {

    private NormalizedColorSensor lLS;
    private NormalizedColorSensor rLS;
    private NormalizedColorSensor rRS;
    private NormalizedColorSensor lRS;

    public static double redThresh=0;
    public static double blueThresh=0;
    public static double greenThresh=0;


    public ColorSensors(HardwareMap hardwareMap) {
        lLS = hardwareMap.get(NormalizedColorSensor.class, "lLS");
        rLS = hardwareMap.get(NormalizedColorSensor.class, "rLS");
        lRS = hardwareMap.get(NormalizedColorSensor.class, "lRS");
        rRS = hardwareMap.get(NormalizedColorSensor.class, "rRS");

    }
    // -1=nothing detected
// 0 =purple
// 1 =green
    public int getColorLeft() {
        NormalizedRGBA colorsL = lLS.getNormalizedColors();
        NormalizedRGBA colorsR = rLS.getNormalizedColors();
        if (colorsL.blue > colorsL.green) {
            if (colorsL.blue > blueThresh) {
                return 1;
            } else {
                return 0;
            }
        } else if ((colorsR.red + colorsR.blue) / 2 > colorsR.green){
            if (colorsR.red > redThresh && colorsR.blue > blueThresh) {
                return 1;
            } else {
                return 0;
            }
        } else if(colorsL.green>greenThresh) {
            return 2;
        } else if(colorsR.green>greenThresh) {
            return 2;
        }  else{
            return -1;
        }
    }
    public int getColorRight() {
        NormalizedRGBA colorsL = lRS.getNormalizedColors();
        NormalizedRGBA colorsR = rRS.getNormalizedColors();
        if (colorsL.blue > colorsL.green) {
            if (colorsL.blue > blueThresh) {
                return 1;
            } else {
                return 0;
            }
        } else if (colorsR.blue > colorsR.green){
            if (colorsR.blue > blueThresh) {
                return 1;
            } else {
                return 0;
            }
        } else if(colorsL.green>greenThresh) {
            return 2;
        } else if(colorsR.green>greenThresh) {
            return 2;
        }  else{
            return 0;
        }
    }

    public NormalizedRGBA[] getColorsBro(){
        NormalizedRGBA[] colors = new NormalizedRGBA[4];
        colors[0]=lLS.getNormalizedColors();
        colors[1]=rLS.getNormalizedColors();
        colors[2]=lRS.getNormalizedColors();
        colors[3]=rRS.getNormalizedColors();
        return colors;
    }

}