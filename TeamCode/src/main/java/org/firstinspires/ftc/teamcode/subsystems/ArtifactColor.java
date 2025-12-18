package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public enum ArtifactColor {
    GREEN(new double[] {0.28, 1.0, 0.7}),
    PURPLE (new double[] {0.63, 0.74, 1.0}),
    NONE   (new double[] {0,0,0});

    public final double[] color;
    ArtifactColor(double[] color) {
        this.color = Utils.normalizeArray(color);
    }
    public static double[] NormalizedRGBAToArray(NormalizedRGBA NRGBA) {
        return Utils.normalizeArray(new double[] {
                (double)NRGBA.red,
                (double)NRGBA.green,
                (double)NRGBA.blue});
    }
}
