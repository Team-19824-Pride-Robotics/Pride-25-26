package org.firstinspires.ftc.teamcode.V1.subsystems;

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

    public flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "fW");
        flywheelB = hardwareMap.get(DcMotorEx.class, "fWB");

    }

    public void init() {
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
//        flywheelB.setDirection(DcMotorEx.Direction.REVERSE);
    }




    public double getVelocity() {
        flywheelVelocity=flywheel.getVelocity();
        return flywheelVelocity;
    }

    public void update(double launchPower) {
        if (launchPower != 0 && launchPower>getVelocity()-60) {
            flywheel.setPower(pidf.calculate(getVelocity(), launchPower));
            flywheelB.setPower(pidf.calculate(getVelocity(), launchPower));
        } else {
            flywheel.setPower(0);
            flywheelB.setPower(0);
        }
    }
    public void setPower(double power){
        flywheel.setPower(-power);
        flywheelB.setPower(-power);
    }


}
