function [ ] = ScoreDistribution( net, path )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[classes, scores] = classify_all(net, path);
positives = [];
negatives = [];
for i = 1:size(classes,2)
    if (classes(i) == 1)
        negatives(end+1) = scores(i);
    else
        positives(end+1) = scores(i);
    end
end
figure('Name','Positives');
hist(positives,20);
figure('Name','Negatives');
hist(negatives,20);
end

