function P_est = triangulation_2(p0,p1,SE_cf,SE_wf,K)
Rcf = SE_cf(1:3,1:3);
Tcf = SE_cf(1:3,4);

A = [K\p0, -Rcf'*(K\p1)];
b = -Rcf'*Tcf;

lambda = (A' * A) \ (A' * b);
P_est1 = (K\p0).*lambda(1);
P_est2 = Rcf'*((K\p1).*lambda(2)-Tcf);
P_est = (P_est1 + P_est2)./2;
P_est = P_est1;

P_est = SE_wf*[P_est;1];
% P_est = P_est(1:3);

end