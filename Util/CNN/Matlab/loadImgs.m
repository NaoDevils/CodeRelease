function [ imdb ] = loadImgs( path, img_size, ratio )
%loadImgs Load all image in path
%   Searches in path for all dirs '0' and '1'
%   and loads all images. 0 is no ball, 1 is ball
%   size The target size for each image in pixel
%   ratio The ration of balls to not balls. If there
%         are more balls, then less no balls will be
%         loaded

i = 1;
ok = 0;
for class=1:-1:0
    dircount = 1;
    fprintf('\nLoad files for class %i...\n', class);
    [status,findout]=system(sprintf('find %s -name %i -type d', path, class));
    dirs = strread(findout, '%s', 'delimiter', sprintf('\n'));
    for dir_ = dirs'
        if size(ok) < dircount
            ok(dircount) = 0;
        end
        msg = '';
        files = [];
        fprintf('\nLoad from %s...\n', dir_{1});
        files = cat(1,files,dir(strcat(dir_{1},'/*.png')));
        for file = files'
            imdb.images.source{i} = 0;
            imdb.images.path{i} = strcat(dir_{1},'/', file.name);
            imdb.images.data(:,:,i) = im2single(imresize(imread(imdb.images.path{i}),[img_size img_size],'nearest'));
            imdb.images.label(i) = class + 1;
            imdb.images.id(i) = i;
            imdb.images.set(i) = (rand<0.1)+1;
            i = i + 1;
            if class == 1
                orig_im = imdb.images.data(:,:,i-1);
                for angle = 90:90:270
                    imdb.images.data(:,:,i) = imrotate(orig_im, angle);
                    imdb.images.source{i} = 1;
                    imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                    imdb.images.id(i) = i;
                    imdb.images.set(i) = (rand<0.1)+1;
                    imdb.images.label(i) = class + 1;
                    i = i + 1;
                end
                %increase image brightness
                imdb.images.data(:,:,i) = imadjust(orig_im,[],[],0.7);
                imdb.images.source{i} = 1;
                imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                imdb.images.id(i) = i;
                imdb.images.set(i) = (rand<0.1)+1;
                imdb.images.label(i) = class + 1;
                i = i + 1;
                
                %decrease image brightness
                imdb.images.data(:,:,i) = imadjust(orig_im,[],[],1.3);
                imdb.images.source{i} = 1;
                imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                imdb.images.id(i) = i;
                imdb.images.set(i) = (rand<0.1)+1;
                imdb.images.label(i) = class + 1;
                i = i + 1;
                
                %gauss filtering
                %imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                %imdb.images.data(:,:,i) = imgaussfilt(orig_img, [1.5 0.01]);
                %imdb.images.source{i} = 1;
                %imdb.images.id(i) = i;
                %imdb.images.set(i) = (rand<0.1)+1;
                %imdb.images.label(i) = class + 1;
                %i = i + 1;
                
                %imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                %imdb.images.data(:,:,i) = imgaussfilt(orig_img, [0.01 1.5]);
                %imdb.images.source{i} = 1;
                %imdb.images.id(i) = i;
                %imdb.images.set(i) = (rand<0.1)+1;
                %imdb.images.label(i) = class + 1;
                %i = i + 1;
                
                %imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                %imdb.images.data(:,:,i) = imgaussfilt(orig_img, [1 0.01]);
                %imdb.images.source{i} = 1;
                %imdb.images.id(i) = i;
                %imdb.images.set(i) = (rand<0.1)+1;
                %imdb.images.label(i) = class + 1;
                %i = i + 1;
                
                %imdb.images.path{i} = strcat(dir_{1},'/', file.name);
                %imdb.images.data(:,:,i) = imgaussfilt(orig_img, [0.01 1]);
                %imdb.images.source{i} = 1;
                %imdb.images.id(i) = i;
                %imdb.images.set(i) = (rand<0.1)+1;
                %imdb.images.label(i) = class + 1;
                %i = i + 1;

            end
            for j=1:length(msg)
                fprintf('\b');
            end
            msg = sprintf('Loading %i', i);
            if (ratio>0)
                if class == 1
                    ok(dircount) = ok(dircount) + ratio;
                end
                if class == 0
                    ok(dircount) = ok(dircount) - 1;
                    if  ok(dircount) <= 0
                        break;
                    end
                end
            end
            fprintf(msg);
        end 
        dircount = dircount + 1;
    end
end
end

