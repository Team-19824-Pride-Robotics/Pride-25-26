package org.firstinspires.ftc.teamcode.omeN_code.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class Agitator {

    private final CRServo agitator;

    private double agitatingPower = 0;
    private double lastPower=0;
    private static double leftSpin=1;
    private static double rightSpin=-1;
    private static double noSpin=0;



    public Agitator(HardwareMap hardwareMap) {
        agitator = hardwareMap.get(CRServo.class, "a");
    }

    public void spinLeft(){
        agitatingPower=leftSpin;
    }
    public void spinRight(){
        agitatingPower=rightSpin;
    }
    public void stop(){
        agitatingPower=noSpin;
    }
    public double getPower(){return agitatingPower;}

    public void update() {
        if(Math.abs(agitatingPower-lastPower)>0.2) {
            lastPower=agitatingPower;
            agitator.setPower(agitatingPower);
        }
    }
}
