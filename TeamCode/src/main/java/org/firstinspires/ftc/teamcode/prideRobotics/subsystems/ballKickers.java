package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ballKickers {

    // Dashboard-tunable positions
    public static double downPosition = 0.15; //Not set
    public static double upPosition  = 0; //Not set

    private final ServoImplEx leftKicker;
    private final ServoImplEx rightKicker;
    private final AnalogInput leftKickerEncoder;
    private final AnalogInput rightKickerEncoder;

    private double desiredPosition;

    public ballKickers(HardwareMap hardwareMap) {
        this.leftKicker  = hardwareMap.get(Servo.class, "flap");
        this.rightKicker  = hardwareMap.get(Servo.class, "flap");
        this.leftKickerEncoder  = hardwareMap.get(Servo.class, "flap");
        this.rightKickerEncoder  = hardwareMap.get(Servo.class, "flap");

    }

