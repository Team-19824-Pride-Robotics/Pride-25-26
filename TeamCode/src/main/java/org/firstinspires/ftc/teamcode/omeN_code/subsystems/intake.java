package org.firstinspires.ftc.teamcode.omeN_code.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class intake {

    private final DcMotor intake;
    private Agitator agitator;

    public double intakingPower = 0;
    public double lastPower=0;



    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "i");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator = new Agitator(hardwareMap);
    }

    public void setPower(double power){
        intakingPower=power;
    }


    public void update(int count, int side) {
        if(Math.abs(intakingPower-lastPower)>0.2) {
            lastPower=intakingPower;
            intake.setPower(intakingPower);
        }


        if(intakingPower<0.1) {
            if (count == 1) {
                if (side == 1) {
                    agitator.spinRight();
                } else {
                    agitator.spinLeft();
                }
            } else {
                agitator.stop();
            }
        }else{
            if(count==2){
                agitator.spinLeft();
            }
        }
    }
}
