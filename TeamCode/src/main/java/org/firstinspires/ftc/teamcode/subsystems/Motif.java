package org.firstinspires.ftc.teamcode.subsystems;

public enum Motif {
    GPP(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE),
    PGP(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE),
    PPG(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN),
    NONE(ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE);
    public ArtifactColor[] sequence;

    private Motif(ArtifactColor color0, ArtifactColor color1, ArtifactColor color2) {
        sequence = new ArtifactColor[3];
        sequence[0] = color0;
        sequence[1] = color1;
        sequence[2] = color2;
    }

    public Motif getNext() {
        if (this == GPP) {
            return PGP;
        }
        else if (this == PGP) {
            return PPG;
        }
        else if (this == PPG) {
            return GPP;
        }
        return this;
    }
}
