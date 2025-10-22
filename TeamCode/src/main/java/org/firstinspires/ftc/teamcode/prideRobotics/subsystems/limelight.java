package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class limelight {

    private final Limelight3A limelight;

    public double intakingPower = 0;

    public limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "lL");
    }
    public void init(){
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public int scanAuto(){
        //int returned equals index of green in motif (0=GPP, 1=PGP, 2=PPG)
        limelight.pipelineSwitch(0);
//        while (result.getPipelineIndex()!=0) {
//        }
//        else if(result != null && result.getPipelineIndex()==1) {
//            pattern="Purple Green Purple";
//        }
//        else if(result != null && result.getPipelineIndex()==2) {
//            pattern="Purple Purple Green";
//        }
//    }
//
//
//    public void update(double launchPower) {
//        intake.setPower(intakingPower);
//    }
}
