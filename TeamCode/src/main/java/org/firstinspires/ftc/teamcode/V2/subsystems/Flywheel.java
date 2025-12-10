package org.firstinspires.ftc.teamcode.V2.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.powerable.SetPower;

public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private static double P = 0.005;
    private static double I = 0;
    private static double D = 0;
    private Flywheel() { }

    private final MotorGroup flywheelMotors = new MotorGroup(
            new MotorEx("fW"),
            new MotorEx("fWB")
    );

    private final ControlSystem flywheelControlSystem = ControlSystem.builder()
            .velPid(P, I, D)
            .build();

    @Override
    public void initialize() {
        new SetPower(flywheelMotors, 0).schedule();
    }

    @Override
    public void periodic() {
        // optional telemetry or monitoring
    }
    public double getVelocity(){
        return flywheelMotors.getVelocity();
    }
    public void update(int velocity) {
        new RunToVelocity(flywheelControlSystem, velocity).schedule();
    }
}