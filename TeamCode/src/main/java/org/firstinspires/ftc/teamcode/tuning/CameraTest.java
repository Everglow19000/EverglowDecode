package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;

@TeleOp(name="CameraTest")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Camera camera = new Camera(hardwareMap);

        camera.start();

        waitForStart();

        double[] pose = new double[3];

        Actions.runBlocking(camera.getFindLocationAction(pose, 50));

        while (opModeIsActive()) {
            telemetry.addData("x", pose[0]);
            telemetry.addData("y", pose[1]);
            telemetry.addData("heading", pose[2]);
            telemetry.update();
        }
    }
}
