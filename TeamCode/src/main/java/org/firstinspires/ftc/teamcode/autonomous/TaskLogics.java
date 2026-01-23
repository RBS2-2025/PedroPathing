package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Util;

public class TaskLogics {
    DcMotor intakeDc, outtakeDc;
    TaskStatus status = TaskStatus.IDLE;
    public TaskLogics(HardwareMap hardwareMap){
        intakeDc = hardwareMap.dcMotor.get("intakeDc");
        outtakeDc = hardwareMap.dcMotor.get("outtakeDc");
    }

    public void loop(){
        switch (status){
            case IDLE:
                //IDLE
                break;
            case INTAKE:
                //TODO intake
                Util.delay(1);
                break;
            case OUTTAKE:
                //TODO outtake
                break;
        }
    }
}

enum TaskStatus{
    IDLE,
    INTAKE,
    OUTTAKE
}