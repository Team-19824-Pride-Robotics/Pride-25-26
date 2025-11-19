package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.arcrobotics.ftclib.controller.PDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Configurable
public class distanceSensors {

    private DistanceSensor lMS; //left middle sensor
    private DistanceSensor rMS; //right middle sensor
    private DistanceSensor fS; //front sensor

    public static double midThreshOne=0; //threshhold for one artifact
    public static double midThreshTwo=0; //threshhold for two
    public static double frontThresh=0;


    public distanceSensors(HardwareMap hardwareMap) {
        lMS = hardwareMap.get(DistanceSensor.class, "lMS");
        rMS = hardwareMap.get(DistanceSensor.class, "rMS");
        fS = hardwareMap.get(DistanceSensor.class, "fS");


    }
    // -1=nothing detected
// 0 =purple
// 1 =green
    public int findMidArtifacts() {
        if(lMS.getDistance(DistanceUnit.INCH)+rMS.getDistance(DistanceUnit.INCH)>midThreshOne){
            return 0;
        } else if((lMS.getDistance(DistanceUnit.INCH)+rMS.getDistance(DistanceUnit.INCH)<midThreshOne)&&(lMS.getDistance(DistanceUnit.INCH)+rMS.getDistance(DistanceUnit.INCH)>midThreshTwo)){
            return 1;
        }  else{
            return 2;
        }
    }
    public double[] getDistances() {
        double[] distances = new double[3];
        distances[0]=lMS.getDistance(DistanceUnit.INCH);
        distances[1]=rMS.getDistance(DistanceUnit.INCH);
        distances[2]=fS.getDistance(DistanceUnit.INCH);
        return distances;
    }







}
