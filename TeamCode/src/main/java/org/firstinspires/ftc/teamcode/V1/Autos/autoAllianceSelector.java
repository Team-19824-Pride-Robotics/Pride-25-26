package org.firstinspires.ftc.teamcode.V1.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static dev.nextftc.ftc.ActiveOpMode.isStopRequested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autoAllianceSelector extends LinearOpMode {
    public boolean redAlliance=true;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                redAlliance=true;
            }
            if(gamepad1.b){
                redAlliance=false;
            }
            telemetry.addData("Press a for red and b for blue", "");
            telemetry.addData("Red alliance", redAlliance);
            telemetry.addData("Blue alliance", !redAlliance);

        }
    }
    public boolean getAlliance(){
        return redAlliance;
    }

}
