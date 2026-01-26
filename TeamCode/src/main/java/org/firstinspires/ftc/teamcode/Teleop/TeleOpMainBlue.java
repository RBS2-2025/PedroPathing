package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TaskLogics;
import org.firstinspires.ftc.teamcode.movement.IMUDriving;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

@TeleOp(name = "TeleOp Blue",group = "TeleOp")
public class TeleOpMainBlue extends OpMode {
    Robot robot;
    IMUDriving imuDriving;
    PanelsHelper panel;
    TaskLogics task;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap,true);
        this.panel = new PanelsHelper(this);
        this.imuDriving = new IMUDriving(hardwareMap,panel,gamepad1);
        this.task = new TaskLogics(this.robot,true);
    }

    @Override
    public void start(){
        this.task.start();
    }

    @Override
    public void loop() {
        imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
        this.task.loop();
    }



}
