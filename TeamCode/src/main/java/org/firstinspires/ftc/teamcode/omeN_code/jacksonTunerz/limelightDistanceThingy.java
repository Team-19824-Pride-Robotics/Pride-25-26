package org.firstinspires.ftc.teamcode.omeN_code.jacksonTunerz;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;

@TeleOp
@Configurable
public class limelightDistanceThingy extends LinearOpMode {
    //mech subsystem declarations
    private limelight limelight;



    @Override
    public void runOpMode() throws InterruptedException {

        limelight = new limelight(hardwareMap);


        //init mechs
        limelight.init();
        limelight.setPipeline(3);




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
        limelight.init();
        limelight.setPipeline(3);





            telemetry.addData("Distance From Goal: ", limelight.getDistance());
            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
