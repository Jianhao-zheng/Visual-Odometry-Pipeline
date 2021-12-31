function [features, valid_key_points] = detectkeypoints(image, method)
%%% defalut method is harris
if nargin == 1
    method = 'Surf';
end

switch method
    case 'Harris'
        corners = detectHarrisFeatures(image);
        [features, valid_key_points] = extractFeatures(image, corners);
    case 'Surf'
        corners = detectSURFFeatures(image,'MetricThreshold',1500);
        [features, valid_key_points] = extractFeatures(image, corners);
end

end