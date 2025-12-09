package org.firstinspires.ftc.teamcode.V1.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;



@Configurable
public class limelight {

    private final Limelight3A limelight;
    private LLResult result;
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
    public int scanAuto() {

        //int returned equals index of green in motif (0=GPP, 1=PGP, 2=PPG)
        int pattern = 0;
        result = limelight.getLatestResult();
        for (int i = 0; i < 3; i++) {
            limelight.pipelineSwitch(i);
                while (result.getPipelineIndex() != i) {
                    result = limelight.getLatestResult();
                    if (result != null && result.getPipelineIndex() == 0) {
                        pattern = i;
                    }
                }
            }

        return pattern;
    }
    public double getDistance(){
        result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            return 67.82807 * Math.pow(result.getTa(), -0.5);
        } else{
            return -1;
        }
    }
    public double getAngle(){
        if(result != null){
            return result.getTxNC();
        } else{
            return -1;
        }
    }

}
