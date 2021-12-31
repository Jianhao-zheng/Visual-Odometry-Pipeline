function scores = shi_tomasi(img, patch_size)

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');
Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

trace = sIxx + sIyy;
determinant = sIxx .* sIyy - sIxy .^ 2;

% the eigen values of a matrix M=[a,b;c,d] are 
% lambda1/2 = (Tr(A)/2 +- ((Tr(A)/2)^2-det(A))^.5
% The smaller one is the one with the negative sign
scores = trace/2 - ((trace/2).^2 - determinant).^0.5;
scores(scores<0) = 0;

scores = padarray(scores, [1+pr 1+pr]);

end
