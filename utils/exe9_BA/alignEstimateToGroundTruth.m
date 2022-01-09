function p_G_C = alignEstimateToGroundTruth(...
    pp_G_C, p_V_C)
% Returns the points of the estimated trajectory p_V_C transformed into the
% ground truth frame G. The similarity transform Sim_G_V is to be chosen
% such that it results in the lowest error between the aligned trajectory
% points p_G_C and the points of the ground truth trajectory pp_G_C. All
% matrices are 3xN.

% Initial guess is identity.
twist_guess = HomogMatrix2twist(eye(4));
scale_guess = 1;

x = [twist_guess; scale_guess];
% Using an external error function for nicer code. Binding pp_G_C and p_V_C
% by casting the function as a function of the hidden state only.
error_terms = @(x) alignError(x, pp_G_C, p_V_C);
options = optimoptions(@lsqnonlin, 'Display', 'iter');
x_optim = lsqnonlin(error_terms, x, [], [], options);

T_G_V = twist2HomogMatrix(x_optim(1:6));
scale_G_V = x_optim(7);

num_frames = size(p_V_C, 2);
p_G_C = scale_G_V * T_G_V(1:3, 1:3) * p_V_C ...
    + repmat(T_G_V(1:3, 4), [1 num_frames]);

end

