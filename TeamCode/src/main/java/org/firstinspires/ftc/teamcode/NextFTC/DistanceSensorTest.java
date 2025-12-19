package org.firstinspires.ftc.teamcode.NextFTC;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.NextFTC.subsystems.DistanceSensors;
@TeleOp
@Configurable
public class DistanceSensorTest extends LinearOpMode {
    //mech subsystem declarations

    private DistanceSensors distanceSensors;
    //fun variables





    @Override
    public void runOpMode() throws InterruptedException {



        distanceSensors = new DistanceSensors(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Count", distanceSensors.getCount());
            telemetry.addData("Side", distanceSensors.getSide());
            telemetry.addData("Raw data Left", distanceSensors.rawVals()[0]);
            telemetry.addData("Raw data Right", distanceSensors.rawVals()[1]);
            telemetry.addData("Raw data Intake", distanceSensors.rawVals()[2]);

            telemetry.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
