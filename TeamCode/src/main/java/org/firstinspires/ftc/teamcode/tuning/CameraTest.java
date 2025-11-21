package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;

import java.util.Arrays;

@TeleOp(name="CameraTest")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Camera camera = new Camera(hardwareMap);

        camera.start();

        waitForStart();

        double[] location = new double[3];

        while (opModeIsActive()) {
            if (gamepad1.circle){
                Actions.runBlocking(camera.getFindLocationAction(location , 50));
            }
            telemetry.addData("location", Arrays.toString(location));
            telemetry.update();
        }
    }
}
