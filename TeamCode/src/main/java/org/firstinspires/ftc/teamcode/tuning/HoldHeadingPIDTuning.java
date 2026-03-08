package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.everglow_library.RotationSquIDFController;

@TeleOp(name="HoldHeadingPIDTuning", group="Tests")
@Config
public class HoldHeadingPIDTuning extends LinearOpMode {
    public static double P = 0.83;
    public static double I = 0;
    public static double D = 0.08;
    public static double F = 0;
    private Rotation2d targetRotation = Rotation2d.exp(0);
    private RotationSquIDFController controller = new RotationSquIDFController(P, I, D, F);
    private boolean isSwitchToTarget = false;
    double lastSwitchTime = -10;
    double angVel;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        controller.setTolerance(0.001);

        waitForStart();

        while (opModeIsActive()) {
            controller.setPIDF(P, I, D, F);
            if (getRuntime() - lastSwitchTime > 5) {
                lastSwitchTime = getRuntime();
                if (isSwitchToTarget) {
                    targetRotation = Rotation2d.exp(Math.PI/2);
                }
                else {
                    targetRotation = Rotation2d.exp(Math.PI/2 + ((2*Math.random()-1)*Math.PI*0.75));
                }
                isSwitchToTarget = !isSwitchToTarget;
            }


            drive.updatePoseEstimate();
            angVel = controller.calculate(drive.localizer.getPose().heading.toDouble(), targetRotation.toDouble());
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0, 0),
                            Math.signum(angVel)*Math.sqrt(Math.abs(angVel))
                    )
            );

            telemetry.addData("curr head", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("target head", targetRotation.toDouble());
            telemetry.addData("error", controller.getPositionError());
            telemetry.addData("acceptable", Math.abs(controller.getPositionError()) < 0.01);
            telemetry.addData("angVel", angVel);
            telemetry.update();
        }
    }
}
