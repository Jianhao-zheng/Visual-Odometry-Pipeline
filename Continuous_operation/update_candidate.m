function S = update_candidate(S,valid_key_candidates,image,K,hyper_paras)
% extract rotation and translation
R_W_C = reshape(S.est_rot(:,end),[3,3]);
t_W_C = S.est_trans(:,end);

% discard redundant new candidate keypoints (whose distance to any
% existing keypoints is less than 'r_discard_redundant')
r = hyper_paras.r_discard_redundant;

new_candidate = valid_key_candidates.Location;

if ~isempty(S.C)
    new_valid_candidate = [];
    for i = 1:size(new_candidate,1)
        u_range = [new_candidate(i,1)-r,new_candidate(i,1)+r];
        v_range = [new_candidate(i,2)-r,new_candidate(i,2)+r];
        check_near_C = S.C(2,:) >= u_range(1) & S.C(2,:) <= u_range(2) &...
                     S.C(1,:) >= v_range(1) & S.C(1,:) <= v_range(2);  % S.C follows [row,col]
        check_near_P = S.P(2,:) >= u_range(1) & S.P(2,:) <= u_range(2) &...
                     S.P(1,:) >= v_range(1) & S.P(1,:) <= v_range(2);  % S.P follows [row,col]
        if sum(check_near_C) == 0 && sum(check_near_P) == 0
            new_valid_candidate = [new_valid_candidate, [new_candidate(i,2);new_candidate(i,1)]];
        end
    end
else
    new_valid_candidate = flipud(valid_key_candidates.Location');
end

% redundant_map = ones(size(image)); 
% for ii = 1:size(S.P,2)
%     redundant_map(max(1,floor(S.P(1,ii)-r)):...
%         min(ceil(S.P(1,ii)+r),size(redundant_map,1)),...
%         max(1,floor(S.P(2,ii)-r)):...
%         min(ceil(S.P(2,ii)+r),size(redundant_map,2))) = 0;
% end
% 
% for ii = 1:size(S.C,2)
%     redundant_map(max(1,floor(S.C(1,ii)-r)):...
%         min(ceil(S.C(1,ii)+r),size(redundant_map,1)),...
%         max(1,floor(S.C(2,ii)-r)):...
%         min(ceil(S.C(2,ii)+r),size(redundant_map,2))) = 0;
% end
% 
% no_discard = ones(size(valid_key_candidates.Location,1),1);
% for ii = 1:size(valid_key_candidates.Location,1)
%     no_discard(ii) = redundant_map(round(valid_key_candidates.Location(ii,2)),round(valid_key_candidates.Location(ii,1)));
% end
% no_discard = logical(no_discard);
% 
% % plot for debugging
% % plot_discard_debug(image,S,valid_key_candidates,no_discard)
% 
% valid_key_candidates = valid_key_candidates(no_discard); 

S.C = [S.C, double(new_valid_candidate)];
S.F = [S.F, double(new_valid_candidate)];
S.T = [S.T, repmat([R_W_C(:);t_W_C(:)],1,size(valid_key_candidates.Location,1))];

end