package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class intake {

    private final DcMotor intake;

    public double intakingPower = 0;



    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");


    }

    public void setPower(double power){
        intakingPower=power;
    }


    public void update(double launchPower) {
        intake.setPower(intakingPower);
    }
}
