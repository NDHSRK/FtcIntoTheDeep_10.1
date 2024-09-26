package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.common.FTCTeleOpDispatch;

@TeleOp(name = "IntoTheDeep", group = "Drive")
//@Disabled
public class IntoTheDeepLaunch extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        FTCTeleOpDispatch.runTeleOp(RobotConstants.RunType.TELEOP, IntoTheDeepTeleOp.class.getSimpleName(), RobotConstants.Alliance.NONE, this,
                (FTCTeleOpDispatch.TeleOpWithAllianceParameters tp) ->
                new IntoTheDeepTeleOp(tp.alliance, tp.linearOpMode, tp.robot));
    }
}