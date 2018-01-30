function [ ] = sortByScore( net, imdb, targetFolder )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

for i = 1:size(imdb.images.data, 3)
    im = 256 * (imdb.images.data(:,:,i) - net.imageMean) ;
    res = vl_simplenn(net, im) ;
    for j=1:size(res(end).x,2)
        [score(j),pred(j)] = max(squeeze(res(end).x(1,j,:))) ;
        fn = sprintf('%s/[%i][%.4f][%i].png',targetFolder,pred(j),score(j),i);
        imwrite(imdb.images.data(:,:,i),fn);
    end
end

end

