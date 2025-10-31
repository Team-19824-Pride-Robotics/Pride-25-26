package org.firstinspires.ftc.teamcode.prideRobotics.subsystems;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class flywheel {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheelB;

    public double flywheelVelocity = 0;
    private static double kP=0;
    private static double kI=0;
    private static double kD=0;
    private static double kF=0;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }




    public double getVelocity() {
        flywheelVelocity=flywheelB.getVelocity();
        return flywheelVelocity;
    }

    public void update(double launchPower) {
        flywheelVelocity=flywheelB.getVelocity();
        flywheelB.setVelocity(launchPower);
        flywheelB.setVelocity(launchPower);
    }


}
