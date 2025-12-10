package org.firstinspires.ftc.teamcode.V2.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    MotorEx intakeMotor;
    private double intakePower=0;
    public Intake() {
         intakeMotor = new MotorEx("i");
    }
    @Override
    public void initialize() {
        new SetPower(intakeMotor, 0).schedule();
    }

    @Override
    public void periodic() {
        // optional telemetry or monitoring
    }

    public void update() {
        new SetPower(intakeMotor, intakePower).schedule();
    }
    public void setPower(double power){
        intakePower=power;
    }
}