# Control_Torque_iiwa_Matlab
Controlar el iiwa simulado en gazebo mediante ros y Matlab


Topics modificados:

  /iiwa_gazebo_plugin/joint_state : Se recibe en él:  -Position : posición articular
                                                      -Velocity: velocidad articular
                                                      -Effort : torque de cada articulación
                                                      -Header.Stamp.seconds: tiempo en segundos
                                                      
  /iiwa_gazebo_plugin/joint_command : Envia los torque que se quieren ejecutar en cada articulación
