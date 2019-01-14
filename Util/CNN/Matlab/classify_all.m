function [ classes, scores ] = classify_all( net, path )
%classify_all Classifies all images in path
%   Classifies all images in path.
%   Use net=train(path); to train a net.
setup;
pattern='\*.png';
files = dir(strcat(path, pattern));
not_ball = {};
ball = {};
classes = [];
scores = [];
for file = files'
    f = strcat(path,'\',file.name);
    [class, score] = classify(net, f);
    classes(end+1) = class;
    scores(end+1) = score;
    if (class == 1)
        not_ball{end+1} = f;
    else
        ball{end+1} = f;
    end
end
%figure;
%montage(ball);
%figure;
%montage(not_ball);
end

