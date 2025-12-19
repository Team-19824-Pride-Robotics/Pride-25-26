package org.firstinspires.ftc.teamcode.V1.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class intake {

    private final DcMotor intake;

    public double intakingPower = 0;



    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "i");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setPower(double power){
        intakingPower=power;
    }


    public void update() {
        intake.setPower(intakingPower);
    }
}
