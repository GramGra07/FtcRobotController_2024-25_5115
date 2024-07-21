object OTOSExtension{
fun initOTOS(val hw: HardwareMap,val startPozd:SparkFun.Pose2d,val name:string):SparkFunOTOS{
    val t = hw.get(SparkFunOTOS::class.java,name)
    t.setLinearUnit(DistanceUnit.INCH);
        t.setAngularUnit(AngleUnit.DEGREES);
        //For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        t.setOffset(offset);
        //For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        t.setLinearScalar(1.0);
        t.setAngularScalar(1.0);
                t.calibrateImu();
                
         t.resetTracking();
         
         SparkFunOTOS.Pose2D currentPosition = startPose
        t.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        t.getVersionInfo(hwVersion, fwVersion);
}
fun SparkFunOTOS.getPose(){
return this.getPosition()
}
fun SparkFunOTOS.Pose2d.toPose(){
return Pose2d(this.x,this.y,this.h)
}
fun SparkFunOTOS.telemetry(val telemetry:Telemetry){
val pose = this.getPose().goPose()
telemetry.addData("x",pose.x)
telemetry.addData("y",pose.y)
telemetry.addData("h",pose.h)
}