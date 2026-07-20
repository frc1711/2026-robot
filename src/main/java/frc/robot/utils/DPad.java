package frc.robot.utils;

public enum DPad {

    LEFT(90),
    RIGHT(-90),
    UP(0),
    DOWN(180);

    private int degrees;

    private DPad(int degrees) {

        this.degrees = degrees;

    }

    public int getDegrees() {

        return this.degrees;

    }

}
