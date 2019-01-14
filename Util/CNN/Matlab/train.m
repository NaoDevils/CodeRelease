function [ net, info ] = train( net, imdb )
%train Train a network
%   net The configuration of the network
%   imdb Images loaded with loadImgs
%   Example: net = train(CNN_xs, loadImgs('Pfad')); 

setup;

trainOpts = net.trainOpts;

imageMean = mean(imdb.images.data(:)) ;
imdb.images.data = imdb.images.data - imageMean ;

[net,info] = cnn_train(net, imdb, @getBatch, trainOpts) ;
net.imageMean = imageMean;
net.layers(end) = [] ;



% --------------------------------------------------------------------
function [im, labels] = getBatch(imdb, batch)
% --------------------------------------------------------------------
im = imdb.images.data(:,:,batch) ;
im = 256 * reshape(im, size(im,1), size(im,2), 1, []) ;
labels = imdb.images.label(1,batch) ;
