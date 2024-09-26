package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsIntoTheDeep;

@Autonomous(name = "BLUE_A4", group = "TeamCode")
//@Disabled
public class BLUE_A4 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        FTCAutoDispatch.runAuto(RobotConstants.RunType.AUTONOMOUS,
                RobotConstantsIntoTheDeep.OpMode.BLUE_A4,
                RobotConstants.Alliance.BLUE, this);
    }
}


