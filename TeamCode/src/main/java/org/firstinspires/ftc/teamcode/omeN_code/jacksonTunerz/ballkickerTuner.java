package org.firstinspires.ftc.teamcode.omeN_code;

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

import org.firstinspires.ftc.teamcode.omeN_code.subsystems.ballKickers;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.colorSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.distanceSensors;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.flywheel;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.intake;
import org.firstinspires.ftc.teamcode.omeN_code.subsystems.limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "ballkickerTuner")
@Configurable

public class ballkickerTuner extends LinearOpMode {


    private ballKickers ballKickers;

    private static double UpRightPos=120;
    private static double UpLeftPos=230;
    private static double DownRightPos=90;
    private static double DownLeftPos=290;

    @Override
    public void runOpMode() throws InterruptedException {

        ballKickers = new ballKickers(hardwareMap);


        ballKickers.retractRight();
        ballKickers.retractLeft();

        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()) {
            if (gamepad1.a) {
                ballKickers.kickBoth();
            }
            if (gamepad1.b) {
                ballKickers.retractBoth();
            }
            ballKickers.update();
        }
    }

}

//green bull party at 4470 lennox blvd. november 3rd 2030
