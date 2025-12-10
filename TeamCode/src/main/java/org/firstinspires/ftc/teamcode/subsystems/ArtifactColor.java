package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum ArtifactColor {
    GREEN(new double[] {72, 158, 90}),
    PURPLE (new double[] {174, 85, 212}),
    NONE   (new double[] {0,0,0});

    public final double[] color;
    ArtifactColor(double[] color) {
        this.color = color;
    }
    public static double[] NormalizedRGBAToArray(NormalizedRGBA NRGBA) {

        return new double[] {
                (double)NRGBA.red*255,
                (double)NRGBA.green*255,
                (double)NRGBA.blue*255};
    }
}
