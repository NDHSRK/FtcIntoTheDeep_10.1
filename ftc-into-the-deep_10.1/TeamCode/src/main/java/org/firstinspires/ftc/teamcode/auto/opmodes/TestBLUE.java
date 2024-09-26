package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;

@Autonomous(name = "TestBLUE", group = "TeamCode")
//@Disabled
public class TestBLUE extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch.runAuto(RobotConstants.RunType.AUTONOMOUS,
                RobotConstantsIntoTheDeep.OpMode.TEST,
                RobotConstants.Alliance.BLUE, this);
    }

}


