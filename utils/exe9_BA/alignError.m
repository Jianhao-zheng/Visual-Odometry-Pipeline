function error = alignError(x, pp_G_C, p_V_C)
% Given x which encodes the similarity transform Sim_G_V as a concatenation
% of twist and scale, return the error pp_G_C - p_G_C (p_G_C = Sim_G_V * 
% p_V_C) as a single column vector.

T_G_V = twist2HomogMatrix(x(1:6));
scale_G_V = x(7);

num_frames = size(p_V_C, 2);
p_G_C = scale_G_V * T_G_V(1:3, 1:3) * p_V_C ...
    + repmat(T_G_V(1:3, 4), [1 num_frames]);

errors = pp_G_C - p_G_C;

error = errors(:);

end

