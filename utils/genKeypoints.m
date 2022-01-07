function [features, valid_key_points] = genKeypoints(image, method)
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

cfigs = struct('SURF',     struct('func', @detectSURFFeatures,     'params', {{'MetricThreshold', 1500}}, 'support_version', 'R2011b (7.13)'),...
               'Harris',   struct('func', @detectHarrisFeatures,   'params', {{}}, 'support_version', 'R2013a (8.1)'),...
               'MinEigen', struct('func', @detectMinEigenFeatures, 'params', {{}}, 'support_version', 'R2013a (8.1)'),...
               'FAST',     struct('func', @detectFASTFeatures,     'params', {{}}, 'support_version', 'R2013a (8.1)'),...
               'BRISK',    struct('func', @detectBRISKFeatures,    'params', {{}}, 'support_version', 'R2014a (8.3)'),...
               'ORB',      struct('func', @detectORBFeatures,      'params', {{}}, 'support_version', 'R2019a (9.6)'),...
               'SIFT',     struct('func', @detectSIFTFeatures,     'params', {{}}, 'support_version', 'R2021b (9.11)'));

cfig_method = cfigs.(method);
surpport_release = regexp(cfig_method.support_version,'\d*\.\d*','match');

if verLessThan('matlab',surpport_release{1})
    error('Error: Current MATLAB version%s does''t support %s. Start to support from %s!', ...
          matlab_release, method, cfig_method.support_version)
else
    points = cfig_method.func(image, cfig_method.params{:});
    [features, valid_key_points] = extractFeatures(image, points); 
end