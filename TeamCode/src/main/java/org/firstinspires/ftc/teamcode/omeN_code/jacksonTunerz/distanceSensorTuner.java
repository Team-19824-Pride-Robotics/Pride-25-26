package org.firstinspires.ftc.teamcode.omeN_code.jacksonTunerz;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.omeN_code.subsystems.distanceSensors;

@TeleOp
@Configurable
public class distanceSensorTuner extends LinearOpMode {
    private distanceSensors distanceSensors;
    double[] distances;

    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensors = new distanceSensors(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

    }
}