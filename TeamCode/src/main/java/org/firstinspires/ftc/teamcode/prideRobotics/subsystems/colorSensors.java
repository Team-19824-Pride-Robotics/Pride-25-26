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

    private NormalizedColorSensor fLS; //front left sensor
    private NormalizedColorSensor fRS; //front right sensor
    private NormalizedColorSensor mLS; //you get the point
    private NormalizedColorSensor mRS;
    private NormalizedColorSensor bLS;
    private NormalizedColorSensor bRS;


    public int[][] ballPos = {
            {0,0},
            {0,0},
            {0,0}
    };
public NormalizedColorSensor[][] sensorPos;
    public static double redThresh=0;
    public static double blueThresh=0;
    public static double greenThresh=0;


    public colorSensors(HardwareMap hardwareMap) {
        bLS = hardwareMap.get(NormalizedColorSensor.class, "bLS");
        bRS = hardwareMap.get(NormalizedColorSensor.class, "bRS");
        mLS = hardwareMap.get(NormalizedColorSensor.class, "mLS");
        mRS = hardwareMap.get(NormalizedColorSensor.class, "mRS");
        fLS = hardwareMap.get(NormalizedColorSensor.class, "fLS");
        fRS = hardwareMap.get(NormalizedColorSensor.class, "fRS");
        sensorPos= new NormalizedColorSensor[][]{
            {fLS, fRS},
            {mLS, mRS},
            {bLS, bRS}
        };
    }
// 0=nothing detected
// 1 =purple
// 2 =green
    public int[][] getBalls() {
     for(int r=0; r<3; r++){
         for(int c=0; c<2; c++){
             ballPos[r][c]=getColor(r,c);
         }
     }
     return ballPos;
    }
    public int getColor(int r, int c) {
        NormalizedRGBA colors = sensorPos[r][c].getNormalizedColors();
        if ((colors.red + colors.blue) / 2 > colors.green) {
            if (colors.red > redThresh && colors.blue > blueThresh) {
                return 1;
            } else {
                return 0;
            }
        } else {
            if (colors.green>greenThresh) {
                return 2;
            } else {
                return 0;
            }
        }
    }







}
