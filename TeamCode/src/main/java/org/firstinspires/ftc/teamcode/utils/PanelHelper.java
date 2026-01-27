package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

// panel 대시보드 사용 도구
public class PanelHelper {
    public final GamepadManager g1Manager;
    public final GamepadManager g2Manager;
    public final TelemetryManager telemetryManager;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private final OpMode op;
    /**
     * Panel 텔레메트리 , 게임패드 init
     * @param opMode
     */
    public PanelHelper(OpMode opMode) {
        this.op = opMode;

        g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
        g2Manager = PanelsGamepad.INSTANCE.getSecondManager();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * 게임패드 동기화 / 반복되야함
     */
    public void updateGamepads() {
        gamepad1 = g1Manager.asCombinedFTCGamepad(op.gamepad1);
        gamepad2 = g2Manager.asCombinedFTCGamepad(op.gamepad2);
    }

    public void debug(String data) {
        telemetryManager.debug(data);
    }

    public void addline(String data) {
        telemetryManager.addLine(data);
    }

    public void addData(String key, Object value) {
        telemetryManager.addData(key, value);
    }


    public void update() {
        telemetryManager.update(op.telemetry);
    }
}
