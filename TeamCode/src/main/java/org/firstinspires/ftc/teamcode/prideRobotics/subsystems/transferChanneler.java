package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class transferChanneler {

    // Dashboard-tunable positions
    public static double leftPosition = 0.55;
    public static double centerPosition = 0.37;
    public static double rightPosition = 0.2;
    private final ServoImplEx channeler;

    private final AnalogInput channelerE;


    private double desiredPosition;

    public transferChanneler(HardwareMap hardwareMap) {
        channeler = hardwareMap.get(ServoImplEx.class, "c");
        channelerE = hardwareMap.get(AnalogInput.class, "cE");
        channeler.setPwmRange(new PwmControl.PwmRange(505, 2495));
    }
    public void coverLeft(){
        desiredPosition=leftPosition;
    }
    public void coverRight(){
        desiredPosition=rightPosition;
    }
    public void center(){
        desiredPosition=centerPosition;
    }
    public double getPos(){
        return channelerE.getVoltage() / 3.3 * 360;
    }
    public double getDesiredPos(){
        return channeler.getPosition();
    }

    public void update(){
        channeler.setPosition(desiredPosition);
    }
}

