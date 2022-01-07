function [features, valid_key_points] = detectkeypoints(image, method)
%%% defalut method is harris
if nargin == 1
    method = 'SURF';
end

% https://en.wikipedia.org/wiki/MATLAB#Release_history
matlab_ver_struct = ver('matlab');
matlab_release = matlab_ver_struct.Release;
% R2011b: 7.13
% R2013a: 8.1
% R2014a: 8.3
% R2019a: 9.6
% R2021b: 9.11

switch method
    case 'SURF'
        if verLessThan('matlab','7.13')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2011b!"])
        else
            % Introduced in R2011b
            points = detectSURFFeatures(image,'MetricThreshold',1500);
            [features, valid_key_points] = extractFeatures(image, points);
        end
    case 'Harris'
        if verLessThan('matlab','8.1')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2013a!"])
        else
            % Introduced in R2013a
            corners = detectHarrisFeatures(image);
            [features, valid_key_points] = extractFeatures(image, corners);
        end
    case 'MinEigen'
        if verLessThan('matlab','8.1')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2013a!"])
        else
            % Introduced in R2013a
            corners = detectMinEigenFeatures(image);
            [features, valid_key_points] = extractFeatures(image, corners);
        end
    case 'FAST'
        if verLessThan('matlab','8.1')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2013a!"])
        else
            % Introduced in R2013a
            points = detectFASTFeatures(image);
            [features, valid_key_points] = extractFeatures(image, points);
        end
    case 'BRISK'
        if verLessThan('matlab','8.3')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2014a!"])
        else
            % Introduced in R2014a
            points = detectBRISKFeatures(image);
            % strongest100 = points.selectStrongest(100);
            % points = BRISKPoints(strongest100);
            [features, valid_key_points] = extractFeatures(image, points);
        end
    case 'ORB'
        if verLessThan('matlab','9.6')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2019a!"])
        else
            % Introduced in R2019a
            points = detectFASTFeatures(image);
            [features, valid_key_points] = extractFeatures(image, points);
        end
    case 'SIFT'
        if verLessThan('matlab','9.11')
            error(["Current MATLAB version" matlab_release ...
                "doesn't support" method "\nStart to support from R2021b!"])
        else
            % Introduced in R2021b
            points = detectSIFTFeatures(image);
            [features, valid_key_points] = extractFeatures(image, points);
        end
end

end