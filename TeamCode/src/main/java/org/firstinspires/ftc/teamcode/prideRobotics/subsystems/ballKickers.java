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
    public static double downLeftPosition = 0.4;
    public static double upLeftPosition = 1;
    public static double downRightPosition = 0.4;
    public static double upRightPosition = 1;

    private final ServoImplEx leftKicker;
    private final ServoImplEx rightKicker;
    private final AnalogInput leftKickerE;
    private final AnalogInput rightKickerE;

    private double desiredLeftPosition;
    private double desiredRightPosition;

    public ballKickers(HardwareMap hardwareMap) {
        leftKicker = hardwareMap.get(ServoImplEx.class, "lK");
        rightKicker = hardwareMap.get(ServoImplEx.class, "rK");
        leftKickerE = hardwareMap.get(AnalogInput.class, "lKE");
        rightKickerE = hardwareMap.get(AnalogInput.class, "rKE");
        leftKicker.setPwmRange(new PwmControl.PwmRange(505, 2495));
        rightKicker.setPwmRange(new PwmControl.PwmRange(505, 2495));
    }
    public void kickLeft(){
        desiredLeftPosition=upPosition;
    }
    public void kickRight(){
        desiredRightPosition=upPosition;
    }
    public void retractLeft(){
        desiredLeftPosition=downPosition;
    }
    public void retractRight(){
        desiredRightPosition=downPosition;
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

