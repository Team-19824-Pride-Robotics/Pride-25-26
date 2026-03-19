package org.firstinspires.ftc.teamcode.omeN_code.jacksonTunerz  ;

import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.allianceSelected;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.pathConstraints;
import static org.firstinspires.ftc.teamcode.omeN_code.FancyAutos.AllianceSelection.redAlliance;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.omeN_code.subsystems.Agitator;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.intake;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "AgitatorTuner")
@Configurable

public class AgitatorTuner extends LinearOpMode {


    private Agitator agitator;

    private static double leftSpin=1;
    private static double rightSpin=-1;
    private static double noSpin=0;

    @Override
    public void runOpMode() throws InterruptedException {

        agitator = new Agitator(hardwareMap);




        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()) {
            if (gamepad1.a) {
                agitator.spinLeft();
            }
            if (gamepad1.b) {
                agitator.spinRight();
            }
            if(!gamepad1.a && !gamepad1.b){
                agitator.stop();
            }
            agitator.update();
            telemetry.addData("Power: ", agitator.getPower());
            telemetry.update();
        }
    }

}

//green bull party at 4470 lennox blvd. november 3rd 2030
