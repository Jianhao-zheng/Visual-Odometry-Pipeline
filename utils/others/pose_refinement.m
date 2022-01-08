function refined_pose = pose_refinement(T_WC, keypoints, landmarks, K)
twist_init = HomogMatrix2twist([T_WC; zeros(1,3) 1]);
f=@(twist)error_func(twist,double(keypoints), double(landmarks), K);
options = optimoptions(@lsqnonlin,'Display','off','MaxIter',600);
optimized_twist = lsqnonlin(f,twist_init,[],[], options);
refined_pose_homo = twist2HomogMatrix(optimized_twist);
refined_pose = refined_pose_homo(1:3,1:4);
end
