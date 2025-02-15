<launch>
    <param name="/use_sim_time" value="true" />

    <arg name="vision" default="true" />

    <arg name="ekf" default="true" />

    <group if="$(arg ekf)">
      <arg name="q_imunominalup_imuup_w" default="1.0" />
      <arg name="q_imunominalup_imuup_x" default="0.0" />
      <arg name="q_imunominalup_imuup_y" default="0.0" />
      <arg name="q_imunominalup_imuup_z" default="0.0" />

      <arg name="q_dvlnominalup_dvlup_w" default="1.0" />
      <arg name="q_dvlnominalup_dvlup_x" default="0.0" />
      <arg name="q_dvlnominalup_dvlup_y" default="0.0" />
      <arg name="q_dvlnominalup_dvlup_z" default="0.0" />

      <arg name="auv_dvl_offset_x" default="0.0" />
      <arg name="auv_dvl_offset_y" default="0.0" />
      <arg name="auv_dvl_offset_z" default="0.0" />

      <param name="q_imunominalup_imuup_w" value="$(arg q_imunominalup_imuup_w)" />
      <param name="q_imunominalup_imuup_x" value="$(arg q_imunominalup_imuup_x)" />
      <param name="q_imunominalup_imuup_y" value="$(arg q_imunominalup_imuup_y)" />
      <param name="q_imunominalup_imuup_z" value="$(arg q_imunominalup_imuup_z)" />

      <param name="q_dvlnominalup_dvlup_w" value="$(arg q_dvlnominalup_dvlup_w)" />
      <param name="q_dvlnominalup_dvlup_x" value="$(arg q_dvlnominalup_dvlup_x)" />
      <param name="q_dvlnominalup_dvlup_y" value="$(arg q_dvlnominalup_dvlup_y)" />
      <param name="q_dvlnominalup_dvlup_z" value="$(arg q_dvlnominalup_dvlup_z)" />

      <param name="auv_dvl_offset_x" value="$(arg auv_dvl_offset_x)" />
      <param name="auv_dvl_offset_y" value="$(arg auv_dvl_offset_y)" />
      <param name="auv_dvl_offset_z" value="$(arg auv_dvl_offset_z)" />

      <node name="dvl_transform_pub" pkg="tf2_ros" type="static_transform_publisher" args="
        $(arg auv_dvl_offset_x) $(arg auv_dvl_offset_y) $(arg auv_dvl_offset_z) 
        $(arg q_dvlnominalup_dvlup_x) $(arg q_dvlnominalup_dvlup_y) $(arg q_dvlnominalup_dvlup_z) $(arg q_dvlnominalup_dvlup_w) 
        auv dvl" />
      <node name="imu_transform_pub" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0 
        $(arg q_imunominalup_imuup_x) $(arg q_imunominalup_imuup_y) $(arg q_imunominalup_imuup_z) $(arg q_imunominalup_imuup_w) 
        auv imu" />
        
      <node pkg="tf2_ros" type="static_transform_publisher" name="depth_transform_pub" args="0 0 0 0 0 0 1 auv depth" />
    </group>

    <include file="$(find bringup)/launch/bringup.launch">
        <arg name="sim" value="true" />
        <arg name="vision" value="$(arg vision)" />
        <arg name="ekf" value="$(arg ekf)" />
    </include>

    <include file="$(find sim)/launch/endpoint.launch"></include>

    <node name="unity_bridge" pkg="sim" type="unity_bridge.py" output="screen">
        <param name="ekf" value="$(arg ekf)" />
    </node>

</launch>
