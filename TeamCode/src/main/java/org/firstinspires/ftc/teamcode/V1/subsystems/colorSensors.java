package org.firstinspires.ftc.teamcode.V1.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@Configurable
public class colorSensors {

    private NormalizedColorSensor lLS;
    private NormalizedColorSensor rLS;
    private NormalizedColorSensor rRS;
    private NormalizedColorSensor lRS;

    public static double redThresh=0.03;
    public static double blueThresh=0;
    public static double greenThresh=0.045;
    public static double objectThresh=0.005;


    public colorSensors(HardwareMap hardwareMap) {
        lLS = hardwareMap.get(NormalizedColorSensor.class, "lLS");
        rLS = hardwareMap.get(NormalizedColorSensor.class, "rLS");
        lRS = hardwareMap.get(NormalizedColorSensor.class, "lRS");
        rRS = hardwareMap.get(NormalizedColorSensor.class, "rRS");

    }
// 0=nothing detected
// 1 =purple
// 2 =green
    public int getColorLeft() {
        NormalizedRGBA colorsL = lLS.getNormalizedColors();
        NormalizedRGBA colorsR = rLS.getNormalizedColors();
        int green = 0;
        int purple = 0;
        if ((colorsL.red > objectThresh || colorsR.red > objectThresh) && (colorsL.blue > objectThresh || colorsR.blue > objectThresh) && (colorsL.green > objectThresh || colorsR.green > objectThresh)) {
//            if (colorsL.red > redThresh || colorsR.red > redThresh) {
//                purple++;
//            } else {
//                green++;
//            }
//            if (colorsL.blue > blueThresh || colorsR.blue > blueThresh) {
//                purple++;
//            } else {
//                green++;
//            }
            if (colorsL.green < greenThresh || colorsR.green < greenThresh) {
                purple++;
            } else {
                green++;
            }
            if (purple > green) {
                return 1;
            } else {
                return 2;
            }
        } else {
            return 0;
        }
    }
    public int getColorRight() {
            NormalizedRGBA colorsL = lRS.getNormalizedColors();
            NormalizedRGBA colorsR = rRS.getNormalizedColors();
            int green=0;
            int purple=0;
            if((colorsL.red > objectThresh || colorsR.red > objectThresh) && (colorsL.blue > objectThresh || colorsR.blue > objectThresh) && (colorsL.green > objectThresh || colorsR.green > objectThresh)){
//                if(colorsL.red>redThresh||colorsR.red>redThresh){
//                    purple++;
//                } else{
//                    green++;
//                }
//                if(colorsL.blue>blueThresh||colorsR.blue>blueThresh){
//                    purple++;
//                } else{
//                    green++;
//                }
                if(colorsL.green<greenThresh&&                                                         colorsR.green<greenThresh){
                    purple++;
                } else{
                    green++;
                }
                if(purple>green){
                    return 1;
                } else{
                    return 2;
                }
            }else{
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
