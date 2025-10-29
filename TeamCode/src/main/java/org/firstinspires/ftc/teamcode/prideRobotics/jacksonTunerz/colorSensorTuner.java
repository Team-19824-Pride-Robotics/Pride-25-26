package org.firstinspires.ftc.teamcode.prideRobotics.jacksonTunerz;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.prideRobotics.subsystems.colorSensors;

@TeleOp
@Configurable
public class colorSensorTuner extends LinearOpMode {
    private colorSensors colorSensors;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensors = new colorSensors(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("-1 = Nothing", "");
            telemetry.addData("0 = Purple", "");
            telemetry.addData("1 = Green", "");
            telemetry.addData("leftColor: ", colorSensors.getColorLeft());
            telemetry.addData("rightColor: ", colorSensors.getColorRight());
            telemetry.update();
        }
    }
}
//stop copying my code, ftc eric/ftc ivan fartenberry
