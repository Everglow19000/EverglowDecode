package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

public class ThreeDeadwheelAndOTOSLocalizer implements Localizer {
    private final OTOSLocalizer otosLocalizer;
    private final ThreeDeadWheelLocalizer deadwheelLocalizer;
    public ThreeDeadwheelAndOTOSLocalizer(OTOSLocalizer otosLocalizer, ThreeDeadWheelLocalizer deadwheelLocalizer) {
        this.otosLocalizer = otosLocalizer;
        this.deadwheelLocalizer = deadwheelLocalizer;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.otosLocalizer.setPose(pose);
        this.deadwheelLocalizer.setPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return deadwheelLocalizer.getPose();
    }

    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d otosVel = otosLocalizer.update();
        PoseVelocity2d deadwheelVel = deadwheelLocalizer.update();

        return deadwheelVel;
    }
}
