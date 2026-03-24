package org.firstinspires.ftc.teamcode.omeN_code.jacksonTunerz;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.Leds;
@TeleOp
@Configurable
public class LedTuner extends LinearOpMode {
    //mech subsystem declarations

    private Leds leds;
    private static double color=0;
    //fun variables





    @Override
    public void runOpMode() throws InterruptedException {



        leds = new Leds(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            leds.setColor(color);
            leds.update();

        }
    }
}
//green bull party at 4470 lennox blvd. november 3rd 2030
