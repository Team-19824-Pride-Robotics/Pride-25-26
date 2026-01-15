package org.firstinspires.ftc.teamcode.V1.jacksonTunerz;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.V1.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.V1.subsystems.limelight;

@TeleOp
@Configurable
public class motifScanner extends LinearOpMode {
    //mech subsystem declarations

    private limelight limelight;



    @Override
    public void runOpMode() throws InterruptedException {


        //Declare mechs
        limelight = new limelight(hardwareMap);
        int motif=0;


        //init mechs
        limelight.init();




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //update mechs

            if(gamepad1.right_bumper){
                motif=limelight.scanAuto();
            }

            telemetry.addData("motif", motif);

            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
