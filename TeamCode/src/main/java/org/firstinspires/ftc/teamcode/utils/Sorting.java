package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging_new;

public class Sorting {

    private LinearOpMode opMode;
    public ActionManaging_new ActionManager;
    private NormalizedColorSensor c1, c2, c3;

    public static final double ROTATE_DURATION_LIFT_SERVO = 0.5; // 초
    public static double test1 = 0.5 - 0.0772;
    public static double test2 = 0.5;
    public static double test3 = 0.5772;

    final float[] hsv1 = new float[3];
    final float[] hsv2 = new float[3];
    final float[] hsv3 = new float[3];

    public Sorting(LinearOpMode opMode, ActionManaging_new actionManager,
                   NormalizedColorSensor c1, NormalizedColorSensor c2, NormalizedColorSensor c3) {
        this.opMode = opMode;
        this.ActionManager = actionManager;
        this.c1 = c1;
        this.c2 = c2;
        this.c3 = c3;
    }

    public void sortForColor(String targetColor) {
        opMode.telemetry.addData("Target Color", targetColor);
        String resultAction = "Scanning...";

        if (ActionManager.needsReset()) {
            opMode.telemetry.addLine("Safety: Resetting Wheel Position");
            ActionManager.resetRotateWheelPosition();
            return;
        }

        if (isTargetColor(c1, targetColor)) {
            opMode.telemetry.addLine("Action: Lift Item from c1");
            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c1";
        } else if (isTargetColor(c2, targetColor)) {
            opMode.telemetry.addLine("Action: Rotate 1 & Lift Item from c2");
            ActionManager.moveWheelRelatively(1);

            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c2";
        } else if (isTargetColor(c3, targetColor)) {
            opMode.telemetry.addLine("Action: Rotate 2 & Lift Item from c3");
            ActionManager.moveWheelRelatively(-1);

            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c3";
        } else {
            resultAction = "None Found";

            if (ActionManager.isNotCentered()) {
                opMode.telemetry.addLine("Auto-Center: Returning to 0.5");
                ActionManager.resetRotateWheelPosition();
            }
        }

        opMode.telemetry.addData("Result", resultAction);
        opMode.telemetry.addData("Wheel Pos", ActionManager.getCurrentWheelPosition());
    }

    public void updateSensorTelemetry() {
        NormalizedRGBA c1c = c1.getNormalizedColors();
        Color.colorToHSV(c1c.toColor(), hsv1);

        NormalizedRGBA c2c = c2.getNormalizedColors();
        Color.colorToHSV(c2c.toColor(), hsv2);

        NormalizedRGBA c3c = c3.getNormalizedColors();
        Color.colorToHSV(c3c.toColor(), hsv3);

        opMode.telemetry.addData("servo", ActionManager.rotateWheel.getPosition());

        opMode.telemetry.addData("C1", detectColorHSV(c1));
        opMode.telemetry.addData("C2", detectColorHSV(c2));
        opMode.telemetry.addData("C3", detectColorHSV(c3));

        opMode.telemetry.addData("C1 h", hsv1[0]);
        opMode.telemetry.addData("C1 s", hsv1[1]);
        opMode.telemetry.addData("C1 v", hsv1[2]);

        opMode.telemetry.addData("C2 h", hsv2[0]);
        opMode.telemetry.addData("C2 s", hsv2[1]);
        opMode.telemetry.addData("C2 v", hsv2[2]);

        opMode.telemetry.addData("C3 h", hsv3[0]);
        opMode.telemetry.addData("C3 s", hsv3[1]);
        opMode.telemetry.addData("C3 v", hsv3[2]);
    }

    private boolean isTargetColor(NormalizedColorSensor sensor, String targetColor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        if (targetColor.equals("Green")) {
            return hsv[1] >= 0.6;
        } else if (targetColor.equals("Purple")) {
            return hsv[0] >= 180;
        }
        return false;
    }

    private String detectColorHSV(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);

        float hue = hsv[0];
        float saturation = hsv[1];

        if (saturation >= 0.6) return "green";
        if (hue >= 180) return "purple";
        return "none";
    }

}