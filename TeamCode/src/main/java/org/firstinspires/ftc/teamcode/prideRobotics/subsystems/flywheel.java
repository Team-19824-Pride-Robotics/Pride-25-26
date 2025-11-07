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
    private static double kP=0.005;
    private static double kI=0;
    private static double kD=0.00001;
    private static double kF=0.00037037037;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "fW");
        flywheelB = hardwareMap.get(DcMotorEx.class, "fWB");

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
        flywheelVelocity = flywheelB.getVelocity();
        if (flywheelVelocity >= launchPower) {
            flywheel.setPower(pidf.calculate(flywheel.getVelocity(), launchPower));
            flywheelB.setPower(pidf.calculate(flywheelB.getVelocity(), launchPower));
        }
    }
    public void setPower(double power){
        flywheel.setPower(power);
        flywheelB.setPower(power);
    }


}
