package org.firstinspires.ftc.teamcode.V1.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class distanceSensors {

    private AnalogInput lS; //left sensor
    private AnalogInput rS; //right sensor

    private AnalogInput iS; //intake sensor
    public static double emptyThresh=0; //todo: tune vals
    public static double oneThresh=0;

    public static double emptyIntakeThresh=0;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    public distanceSensors(HardwareMap hardwareMap) {
        lS = hardwareMap.get(AnalogInput.class, "lDS");
        rS = hardwareMap.get(AnalogInput.class, "rDS");
        iS = hardwareMap.get(AnalogInput.class, "iDS");


    }

    public int getCount() {
        //count in transfer
        double voltsLeft = lS.getVoltage();
        double voltsRight = rS.getVoltage();
        double voltsIntake = iS.getVoltage();
        double distLeft = (voltsLeft / MAX_VOLTS) * MAX_DISTANCE_MM;
        double distRight = (voltsRight / MAX_VOLTS) * MAX_DISTANCE_MM;
        double distIntake = (voltsIntake / MAX_VOLTS) * MAX_DISTANCE_MM;
        int count=0;
        if(distLeft+distRight<emptyThresh){
            if(distLeft+distRight<oneThresh){
                count=count+2;
            } else{
                count++;
            }
        }

        //count in intake
        if(distIntake<emptyIntakeThresh){
            count++;
        }
        return count;
    }
    public int getSide(){
        double voltsLeft = lS.getVoltage();
        double voltsRight = rS.getVoltage();
        if(voltsLeft>voltsRight){
            return 1; //1=right, 0=left
        } else{
            return 0;
        }
    }
    public double[] rawVals(){
        double voltsLeft = lS.getVoltage();
        double voltsRight = rS.getVoltage();
        double voltsIntake = iS.getVoltage();
        double[] returnVal = new double[3];
        returnVal[0] = (voltsLeft / MAX_VOLTS) * MAX_DISTANCE_MM;
        returnVal[1] = (voltsRight / MAX_VOLTS) * MAX_DISTANCE_MM;
        returnVal[2] = (voltsIntake / MAX_VOLTS) * MAX_DISTANCE_MM;
        return returnVal;
    }









}
