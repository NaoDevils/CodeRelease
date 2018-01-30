function [ class, score ] = classify( net, path )
%classify Classifies a single image
%   net The trained CNN
%   path The path of the PNG
im = 256 * (im2single(imresize(imread(path),[16 16],'nearest')) - net.imageMean);
res = vl_simplenn(net, im) ;
for j=1:size(res(end).x,2)
  [score(j),pred(j)] = max(squeeze(res(end).x(1,j,:))) ;
end
class = pred(1);
score = score(1);
end

