package org.firstinspires.ftc.teamcode.prideRobotics.jacksonTunerz;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.distanceSensors;

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

        while (opModeIsActive()) {
            distances = distanceSensors.getDistances();

            telemetry.addData("left middle sensor", distances[0]);
            telemetry.addData("right middle sensor ", distances[1]);
            telemetry.addData("front sensor ", distances[2]);

            telemetry.update();
        }
    }
}
//stop copying my code, ftc eric/ftc ivan fartenberry
