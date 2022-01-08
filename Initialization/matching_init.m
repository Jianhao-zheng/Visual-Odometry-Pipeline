function [init_points,matched_points] = matching_init(img0,img_seqs,hyper_paras)

img_seq_len = length(img_seqs);
img1 = img_seqs{end};

switch hyper_paras.init_matching_method
    case 'KLT'
        KLT_tracker_init = vision.PointTracker(...
            'NumPyramidLevels',5,...
            'MaxBidirectionalError',3);

        [~, valid_key_candidates0] = genKeypoints(img0,hyper_paras.feature_extract,hyper_paras.feature_extract_options);
        init_points_ = valid_key_candidates0.Location;
        initialize(KLT_tracker_init, init_points_, img0);

        for i = 1:img_seq_len-1
            img_intermediate = img_seqs{i};
            [~,~] = KLT_tracker_init(img_intermediate);
        end

        [matched_points1_, validity_img1] = KLT_tracker_init(img1);

        init_points = init_points_(validity_img1,:);
        matched_points = matched_points1_(validity_img1,:);
    case 'Des_match'
        [feat0, valid0] = genKeypoints(img0,hyper_paras.feature_extract,hyper_paras.feature_extract_options);  % 416 x 64
        [feat1, valid1] = genKeypoints(img1,hyper_paras.feature_extract,hyper_paras.feature_extract_options);  % 422 x 64
        % Introduced in R2011a
        indexPairs = matchFeatures(feat0,feat1);

        init_points = valid0(indexPairs(:,1)).Location;
        matched_points = valid1(indexPairs(:,2)).Location;
end

if hyper_paras.show_match_res
    figure; showMatchedFeatures(img0,img1,init_points,matched_points);
    legend('init\_points','matched\_points');
end

end