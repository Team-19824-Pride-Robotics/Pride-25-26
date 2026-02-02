package org.firstinspires.ftc.teamcode.omeN_code.FancyAutos;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.distanceSensors;
@TeleOp
@Configurable
public class AllianceSelection extends LinearOpMode {
    public static boolean redAlliance = false;
    public static boolean allianceSelected = false;

    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.b){
                redAlliance=true;
                allianceSelected=true;
            } if(gamepad1.x){
                redAlliance=false;
                allianceSelected=true;
            }
            telemetry.addData("Press B for red alliance", "");
            telemetry.addData("Press X for blue alliance", "");
            telemetry.addData("red alliance", redAlliance);
            telemetry.addData("blue alliance", !redAlliance);
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
