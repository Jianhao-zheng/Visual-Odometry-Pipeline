function [init_points,matched_points] = matching_init(img0,img_seqs,hyper_paras)
img_seq_len = length(img_seqs);

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

        img1 = img_seqs{end};
        [matched_points1_, validity_img1] = KLT_tracker_init(img1);
        matched_points = matched_points1_(validity_img1,:);

        init_points = init_points_(validity_img1,:);
end


end