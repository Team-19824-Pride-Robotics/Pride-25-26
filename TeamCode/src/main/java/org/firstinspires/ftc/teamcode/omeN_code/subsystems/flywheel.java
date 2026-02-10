package org.firstinspires.ftc.teamcode.omeN_code.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class flywheel {

    private final DcMotorEx flywheel;
    private final DcMotorEx flywheelB;

    public double flywheelVelocity = 0;
    private static double kP=0.015;
    private static double kI=0;
    private static double kD=0.00001;
    private static double kF=0.00068;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    double lastPower=0;

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
        flywheelVelocity=flywheel.getVelocity();
        return flywheelVelocity;
    }

    public void update(double launchPower) {
        if (launchPower != 0 && launchPower>getVelocity()-60) {
            double pidfPower = pidf.calculate(getVelocity(), launchPower);
            if(Math.abs(pidfPower-lastPower)>0.02) {
                flywheel.setPower(pidfPower);
                flywheelB.setPower(pidfPower);
                lastPower=pidfPower;
            }
        } else {
            if(lastPower!=0) {
                flywheel.setPower(0);
                flywheelB.setPower(0);
                lastPower=0;
            }
        }
    }
    public void setPower(double power){
        if(Math.abs(lastPower-power)>0.02) {
            flywheel.setPower(-power);
            flywheelB.setPower(-power);
            lastPower=power;
        }
    }


}
