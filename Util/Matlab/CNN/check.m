function [ ] = check( net, imdb )
%check Check filter and resulting dimensions 
%   This can be done on an trained net

net.trainOpts.numEpochs = 0 ;
net = train(net, imdb);
im = 256 * (imdb.images.data(:,:,1) - net.imageMean) ;
res = vl_simplenn(net, im) ;
for i=1:size(res,2)
    Img_X(i,1)=size(res(i).x,1);
    Img_Y(i,1)=size(res(i).x,2);
    Img_Z(i,1)=size(res(i).x,3);
end
for i=1:size(net.layers,2)
    if size(net.layers{1,i}.weights,1) > 0
     Filter_X(i,1) = size(net.layers{1,i}.weights{1,1},1);
     Filter_Y(i,1) = size(net.layers{1,i}.weights{1,1},2);
     Filter_Z(i,1) = size(net.layers{1,i}.weights{1,1},3);
     Filter_Num(i,1) = size(net.layers{1,i}.weights{1,1},4);
     Pad(i,1) = net.layers{1,i}.pad;
     Stride(i,1) = net.layers{1,i}.stride;
    else
     Filter_X(i,1) = 0;
     Filter_Y(i,1) = 0;
     Filter_Z(i,1) = 0;
     Filter_Num(i,1) = 0;
     Pad(i,1) = 0;
     Stride(i,1) = 0;
    end
    if strcmp(net.layers{1,i}.type, 'pool') == 1
     Filter_X(i,1) = net.layers{1,i}.pool(1);
     Filter_Y(i,1) = net.layers{1,i}.pool(2);  
     Filter_Z(i,1) = 1;
     Pad(i,1) = net.layers{1,i}.pad;
     Stride(i,1) = net.layers{1,i}.stride;
    end
    if strcmp(net.layers{1,i}.type, 'relu') == 1
     Filter_X(i,1) = 1;
     Filter_Y(i,1) = 1; 
     Filter_Z(i,1) = 1;
    end
    Type{i,1} = net.layers{1,i}.type;
end
Filter_X(end+1,1)=0;
Filter_Y(end+1,1)=0;
Filter_Z(end+1,1)=0;
Filter_Num(end+1,1)=0;
Type{end+1,1}=' ';
Pad(end+1,1) = 0;
Stride(end+1,1) = 0;
disp(table(Img_X,Img_Y,Img_Z,Type,Filter_X,Filter_Y,Filter_Z,Filter_Num, Pad,Stride))

end

