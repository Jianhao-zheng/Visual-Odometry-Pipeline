function P = linearTriangulation_refinement(P_init,p1,p2,M1,M2)
num_points = size(P_init,2);
P = P_init;
for i = 1:num_points
    P_W = P_init(1:3,i);
    error_terms = @(P) [reprojection_error(P,p1(1:2,i),M1); reprojection_error(P,p2(1:2,i),M2)];
    options = optimoptions(@lsqnonlin,'Display', 'off',...
                'MaxIter', 20);
    P(1:3,i) = lsqnonlin(error_terms, P_W, [], [], options);
end

end




% used for nonlinear refinement
function repro_err = reprojection_error(P,p,M)
p_repro = M*[P;1];
p_repro = p_repro(1:2)./p_repro(3);
repro_err = p_repro - p;
end