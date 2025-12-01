package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2)
            .forwardZeroPowerAcceleration(-41.19157413313582)
            .lateralZeroPowerAcceleration(-58.30373587561396)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0001, 0.005, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0,0.0005, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0.0,0.000010,0.6,0.0010));



    public static MecanumConstants driveConstants = new MecanumConstants()
            // Check issue with either wheels or wiring of motors because drive code is not working atm
            .maxPower(1)
            .xVelocity(79.29888723778913)
            .yVelocity(61.73204701341043)




            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
    0.99, 100,
     1,
    1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5) // 3.125   5.500
            .strafePodX(-2) // 0.375  -3.700
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            //If running into issue with pods later keep in mind the x is y and y is x, no clue why but otherwise values wouldnt work
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}