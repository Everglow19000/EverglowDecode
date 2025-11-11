package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

@TeleOp(name="CameraTest")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Camera camera = new Camera(hardwareMap);

        camera.start();

        waitForStart();

        Motif[] motif = new Motif[1];

        Actions.runBlocking(camera.getDetermineMotifAction(motif));

        while (opModeIsActive()) {
            telemetry.addData("motif", motif[0]);
            telemetry.update();
        }
    }
}
