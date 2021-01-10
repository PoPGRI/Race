## Gazebo Model Collision Plugin 

This is a gazebo plugin that needs to be added to your URDF / gazebo files in order to make collision detection of the base work.

Please place
```

  <!-- found name via: gz sdf -p turtlebot3_burger.urdf.xacro > robot.sdf -->
  <gazebo reference="base_footprint">
    <sensor name='collision_sensor' type='contact'>
      <contact>
        <collision>base_footprint_fixed_joint_lump__base_link_collision</collision>
      </contact>
      <plugin name="collision_sensor" filename="libgazebo_collision_sensor.so">
        <rosDebugLevel>na</rosDebugLevel>
        <topicName>/gazebo/base_collision</topicName>
        <updateRate>10.0</updateRate>
      </plugin>
    </sensor>
  </gazebo>
```

in your gazebo.xacro file. The reference frame should be the global frame of your robot in simulation, which is generally `base_footprint`. The name of the collision entry can be found via `gz sdf -p turtlebot3_burger.urdf.xacro > robot.sdf` and reading the `robot.sdf` file to find the name of the collision object for the base link. We care about base link because that is the recommended place to define the collision object for the base robot and inertia information. If you'd like to be conservative or not collision check against the mesh, a collision object like a box surrounding the robot will work as well.

Setting the output topic and update rate are optional. 
