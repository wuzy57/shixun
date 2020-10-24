^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package exhx5
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-10-28)
------------------
Update WalkingControl::calcFootStepParam() and  OnlineWalkingModule::resetBodyPose() and update the init_pose.yaml file in exhx_gui_demo

    if (i == 0 ||
        i == 1 ||
        i == foot_step_size_-1)
    {
      msg.x = 0.0;
      msg.y = foot_origin_shift_y_;
      theta = 0.0;
    }
    if (i == (foot_step_size_-1))
    {
      msg.x = -foot_step_command_.step_length;
      if ((foot_step_size_-1) % 2 ==0 )
      {
        ROS_WARN("Turn Right");
      }
      else
      {
        ROS_WARN("Turn Left");
      }
    }

in OnlineWalkingModule::resetBodyPose()
des_body_pos_[2] = 0.3002256;      //THAT IS IT. CHANGE FROM 0.3402256 to 0.2802256


0.1.0 (2019-08-30)
------------------
* added new metapackage for EXHX5
