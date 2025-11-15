package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ballKickers {

    // Dashboard-tunable positions
    public static double downLeftPosition = 0.8;
    public static double upLeftPosition = 0.6;
    public static double downRightPosition = 0.85;
    public static double upRightPosition = 0.45;

    private final ServoImplEx leftKicker;
    private final ServoImplEx rightKicker;
    private final AnalogInput leftKickerE;
    private final AnalogInput rightKickerE;

    private double desiredLeftPosition;
    private double desiredRightPosition;

    public ballKickers(HardwareMap hardwareMap) {
        leftKicker = hardwareMap.get(ServoImplEx.class, "lBK");
        rightKicker = hardwareMap.get(ServoImplEx.class, "rBK");
        leftKickerE = hardwareMap.get(AnalogInput.class, "lBKE");
        rightKickerE = hardwareMap.get(AnalogInput.class, "rBKE");
        leftKicker.setPwmRange(new PwmControl.PwmRange(505, 2495));
        rightKicker.setPwmRange(new PwmControl.PwmRange(505, 2495));
    }
    public void kickLeft(){
        desiredLeftPosition=upLeftPosition;
    }
    public void kickRight(){
        desiredRightPosition=upRightPosition;
    }
    public void retractLeft(){
        desiredLeftPosition=downLeftPosition;
    }
    public void retractRight(){
        desiredRightPosition=downRightPosition;
    }
    public double getLeftPos(){
        return leftKickerE.getVoltage() / 3.3 * 360;
    }
    public double getRightPos(){
        return rightKickerE.getVoltage() / 3.3 * 360;
    }
    public double getRightDesiredPos(){
        return rightKicker.getPosition();
    }
    public double getLeftDesiredPos(){
        return leftKicker.getPosition();
    }
    public void update(){
        leftKicker.setPosition(desiredLeftPosition);
        rightKicker.setPosition(desiredRightPosition);
    }
}

