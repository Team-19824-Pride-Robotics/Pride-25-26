package org.firstinspires.ftc.teamcode.V2.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class DistanceSensors {

    private AnalogInput lS; //left sensor
    private AnalogInput rS; //right sensor

    private AnalogInput iS; //intake sensor
    public static double emptyThresh=0; //todo: tune vals
    public static double oneThresh=0;

    public static double emptyIntakeThresh=0;
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    public DistanceSensors(HardwareMap hardwareMap) {
        lS = hardwareMap.get(AnalogInput.class, "lS");
        rS = hardwareMap.get(AnalogInput.class, "rS");


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
        if(distLeft+distRight>emptyThresh){
        } else if (distLeft+distRight>oneThresh){
            count++;
        }
        else{
            count=count+2;
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






}
