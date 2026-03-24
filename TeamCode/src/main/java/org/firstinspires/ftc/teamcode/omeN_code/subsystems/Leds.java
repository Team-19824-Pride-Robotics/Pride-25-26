package org.firstinspires.ftc.teamcode.omeN_code.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
public class Leds {

    private Servo leds;
    private double currentColor=0;
    public Leds(HardwareMap hardwareMap) {
        leds = hardwareMap.get(Servo.class, "led");
    }
    public void setColor(double color){
        currentColor=color;
    }
    public void update(){
         leds.setPosition(currentColor);
    }









}
