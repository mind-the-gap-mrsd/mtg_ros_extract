evo_traj kitti bot.kitti --ref vicon_gt.kitti --align_origin --plot --plot_mode xy -v
evo_ape kitti vicon_gt.kitti bot.kitti -v --plot
evo_rpe kitti vicon_gt.kitti bot.kitti -v --plot

evo_traj kitti slam.kitti --ref vicon_gt.kitti --align_origin --plot --plot_mode xy -v
evo_ape kitti vicon_gt.kitti slam.kitti -v --plot
evo_rpe kitti vicon_gt.kitti slam.kitti -v --plot
