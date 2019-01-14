function [ ] = cnncompiler( net, imdb )
%classify_all_reimpl Writes the C Code for the given CNN
%   net The trained network. It is converted to C, compiled
% 	and tested against the Matlab implementation of the
%   network. 
%   imdb The image database

i = 1;
ok = 0;
wrong = 0;
false_positive = 0;
false_negative = 0;
true_positive = 0;
true_negative = 0;
t = 0;
ball = 0;
no_ball = 0;
%c_inf.path = '../../../Src/Modules/Perception/CLIP/cnn.c';
c_inf.path = 'cnn.c';
c_inf.x_dim = size(imdb.images.data(:,:,1), 1);
c_inf.y_dim = size(imdb.images.data(:,:,1), 2);
c_inf = writeHeader(c_inf);

fprintf('Search maximum for normalization...\n');
ball_score = [0];
noball_score = [0];
for i = 1:size(imdb.images.data, 3)
    im = 256 * (imdb.images.data(:,:,i) - net.imageMean) ;
    res = vl_simplenn(net, im) ;
    for j=1:size(res(end).x,2)
      [score(j),pred(j)] = max(squeeze(res(end).x(1,j,:))) ;
    end
    if imdb.images.label(i) == 2
        ball_score(end+1) = score(1);
    else
        noball_score(end+1) = score(1);
    end
end
max_score = max(ball_score);
    
fprintf('Generating C code...\n');
for i = randperm(size(imdb.images.data, 3))
    fprintf('Loop: %i\r', i);
    binfile = -1;
    while binfile == -1
        binfile = fopen('img.bin','w');
    end
    fwrite(binfile, imdb.images.data(:,:,i),'single');
    fclose(binfile);
    im = 256 * (imdb.images.data(:,:,i) - net.imageMean) ;
    label = imdb.images.label(i);
    tic;
    for xi = 1:size(im,1)
        for xj = 1:size(im,2)
            for xk = 1:size(im,3)
                writeC(c_inf, '\tx0[%i][%i][%i] = 256 * (x0[%i][%i][%i] - %ff);\n', ...
                    xi-1,xj-1,xk-1,xi-1,xj-1,xk-1,net.imageMean);
            end
        end
    end 

%     res = vl_simplenn(net, im) ;
    x{1} = im;
    x_orig = im;
    for i=1:size(net.layers,2)
        l = net.layers{1,i};
        switch l.type
            case 'conv'
                if l.pad ~= 0
                    [x{i}, c_inf] = pad(x{i}, l.pad, 'zero', c_inf);      
                end
                x_orig = vl_nnconv(x_orig, ... 
                    l.weights{1}, ...
                    l.weights{2}, ...
                    'pad', l.pad, ...
                    'stride', l.stride) ;
                [x{i+1}, c_inf] = convolution(x{i}, ...
                    l.weights{1}, ...
                    l.weights{2}, ...
                    l.stride, c_inf);
            case 'pool'
                if l.pad ~= 0
                    [x{i}, c_inf] = pad(x{i}, l.pad, 'copy', c_inf);      
                end
                x_orig = vl_nnpool(x_orig, ...
                    l.pool, ...
                    'pad', l.pad, ...
                    'stride', l.stride, ...
                    'method', l.method);
                if l.method == 'avg'
                    [x{i+1}, c_inf] = avg_pool(x{i}, l.pool, l.stride, c_inf);
                end
                if l.method == 'max'
                    [x{i+1}, c_inf] = max_pool(x{i}, l.pool, l.stride, c_inf);
                end
            case 'relu'
                x_orig = vl_nnrelu(x_orig,[]) ;
                [x{i+1}, c_inf] = rectifiedLinearUnit(x{i}, c_inf);
        end
        assert(sum(x{i+1}(:)-x_orig(:)) < 0.01);
    end
    
    t = [t,toc];
    for j=1:size(x{i+1},2)
      [score(j),pred(j)] = max(squeeze(x{i+1}(1,j,:))) ;
      writeC(c_inf, '\tres[%i] = 0;\n',...
      	j-1);
      for ix = 2:size(x{i+1},3)
        writeC(c_inf, '\tres[%i] = x%i[%i][%i][%i] > x%i[%i][%i][res[%i]] ? %i : res[%i];\n',...
            j-1, c_inf.layer-1, 0, j-1, ix-1, ...
            c_inf.layer-1, 0, j-1, j-1, ...
            ix-1, j-1);
      end  
      for ix = 1:size(x{i+1},3)
        writeC(c_inf, '\tscores[%i] = x%i[%i][%i][%i] / %ff;\n',...
      	ix-1, c_inf.layer-1, 0, j-1, ix-1, max_score);
      end
    end
    if pred == label
        ok = ok + 1;
        if (pred == 2)
            true_positive = true_positive + 1;
            true_positives{true_positive} = imdb.images.path{i};
        else
            true_negative = true_negative + 1;
        end
    else
        wrong = wrong + 1;
        if (pred == 2)
            false_positive = false_positive + 1;
            false_positives{false_positive} = imdb.images.path{i};
        else
            false_negative = false_negative + 1;
            false_negatives{false_negative} = imdb.images.path{i};
        end
    end
    if (imdb.images.label(i) == 1)
        no_ball = no_ball + 1;
    else
        ball = ball + 1;
    end
    c_inf = writeFooter(c_inf);
    if system('a.exe')+1 ~= pred
        fprintf('Error\n');
    end
end

fprintf('\r\nOk: %i, Wrong: %i, Error: %f%%\n', ok, wrong, wrong/(ok+wrong)*100);
fprintf('\r\n     \t| Positive\t\t| Negative\n');
fprintf('True \t| %i (%.1f%%)\t| %i (%.1f%%)\n', true_positive, true_positive*100/ball, true_negative, true_negative*100/no_ball);
fprintf('False \t| %i (%.1f%%)\t| %i (%.1f%%)\n', false_positive, false_positive*100/no_ball, false_negative, false_negative*100/ball);
mean(t)
figure('name', 'False Negatives');
montage(false_negatives);
figure('name', 'True Positives');
montage(true_positives);
figure('name', 'False Positives');
montage(false_positives);

function [x_out, c_inf] = convolution(x, w, b, stride, c_inf)
    x_res = ceil((size(x,1) - size(w,1) + 1)/stride);
    y_res = ceil((size(x,2) - size(w,2) + 1)/stride);
    z_res = size(w,4);
    x_out = single(zeros(x_res, y_res, z_res));
    writeC(c_inf, '\tfloat x%i alignas(32) [%i][%i][%i];\n', c_inf.layer, x_res, y_res, z_res);
    for i = 1:size(x_out,1)
        for j = 1:size(x_out,2)
            for kw = 1:size(w,4)
                x_out(i,j,kw) = b(kw);
                writeC(c_inf, '\tx%i[%i][%i][%i] = %ff;\n', c_inf.layer, i-1, j-1, kw-1, b(kw));
            end
        end
    end
    if mod(size(w,4),4) == 0
        writeC(c_inf, '\t{__m128 w, x, y;\n');
        for ix = 1:stride:size(x,1) - size(w,1) + 1
            for jx = 1:stride:size(x,2) - size(w,2) + 1
                for iw = 1:size(w,1)
                    for jw = 1:size(w,2)
                        for kw = 1:size(w,3)
                            for lw = 1:4:size(w,4)
                                x_out_1 = floor(ix/stride+0.5);
                                x_out_2 = floor(jx/stride+0.5);
                                x_1 = ix+iw-1;
                                x_2 = jx+jw-1;
                                x_out(x_out_1,x_out_2,lw)   = single(x_out(x_out_1,x_out_2,lw)   + w(iw,jw,kw,lw)   * x(x_1,x_2,kw));
                                x_out(x_out_1,x_out_2,lw+1) = single(x_out(x_out_1,x_out_2,lw+1) + w(iw,jw,kw,lw+1) * x(x_1,x_2,kw));
                                x_out(x_out_1,x_out_2,lw+2) = single(x_out(x_out_1,x_out_2,lw+2) + w(iw,jw,kw,lw+2) * x(x_1,x_2,kw));
                                x_out(x_out_1,x_out_2,lw+3) = single(x_out(x_out_1,x_out_2,lw+3) + w(iw,jw,kw,lw+3) * x(x_1,x_2,kw));
                                writeC(c_inf,'\tw = _mm_set_ps(%ff, %ff, %ff, %ff);\n', ... 
                                       w(iw,jw,kw,lw+3), w(iw,jw,kw,lw+2), ...
                                       w(iw,jw,kw,lw+1), w(iw,jw,kw,lw));
                                writeC(c_inf,'\tx = _mm_load_ps1(&x%i[%i][%i][%i]);\n', ...
                                    c_inf.layer-1, x_1-1,x_2-1,kw-1);
                                writeC(c_inf,'\ty = _mm_mul_ps(w, x);\n');
                                writeC(c_inf,'\tx = _mm_load_ps((float*)&x%i[%i][%i][%i]);\n',...
                                    c_inf.layer, x_out_1-1, x_out_2-1, lw-1);
                                writeC(c_inf,'\tx = _mm_add_ps(x, y);\n');
                                writeC(c_inf,'\t_mm_store_ps((float*)&x%i[%i][%i][%i], x);\n',...
                                    c_inf.layer, x_out_1-1, x_out_2-1, lw-1);
                            end
                        end
                    end
                end
            end
        end
        writeC(c_inf, '\t}\n');
    else
        for ix = 1:stride:size(x,1) - size(w,1) + 1
            for jx = 1:stride:size(x,2) - size(w,2) + 1
                for iw = 1:size(w,1)
                    for jw = 1:size(w,2)
                        for kx = 1:size(w,3)
                            for kw = 1:size(w,4)
                                x_out_1 = floor(ix/stride+0.5);
                                x_out_2 = floor(jx/stride+0.5);
                                x_1 = ix+iw-1;
                                x_2 = jx+jw-1;
                                x_out(x_out_1,x_out_2,kw) = ...
                                    single(x_out(x_out_1,x_out_2,kw) + ... 
                                    w(iw,jw,kx,kw) * x(x_1,x_2,kx));
                                writeC(c_inf,'\tx%i[%i][%i][%i] += %ff * x%i[%i][%i][%i];\n', ...
                                    c_inf.layer, x_out_1-1, x_out_2-1, kw-1,  w(iw,jw,kx,kw), c_inf.layer-1, ...
                                    x_1-1,x_2-1,kx-1 );
                            end
                        end
                    end
                end
            end
        end
    end
    c_inf.layer = c_inf.layer + 1;

function [x_out, c_inf] = pad(x, pad, method, c_inf)
    if 4 ~= size(pad)
        pad = repmat(pad(1), 2);
    end
    x_res = size(x,1) + pad(1) + pad(2);
    y_res = size(x,2) + pad(3) + pad(4);
    z_res = size(x,3);
    x_out = single(zeros(x_res, y_res, z_res));
    writeC(c_inf, '\tfloat x%i[%i][%i][%i];\n', c_inf.layer, x_res, y_res, z_res);
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            for k = 1:size(x,3)
                x_out_1 = i+pad(1);
                x_out_2 = j+pad(3);
                x_out(x_out_1, x_out_2, k) = x(i, j, k);
                writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n',...
                    c_inf.layer, x_out_1-1, x_out_2-1, k-1, ...
                    c_inf.layer-1, i-1, j-1, k-1);
            end
        end
    end
    if strcmp(method, 'copy')
        for i = 1:pad(1)
            for j = 1:size(x_out,2)
                for k = 1:size(x_out,3)
                    x_out_1 = pad(1)+1;
                    x_out(i,j,k) = x_out(x_out_1,j,k);
                    writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n',...
                        c_inf.layer,i-1, j-1, k-1, ...
                        c_inf.layer, x_out_1-1, j-1, k-1);
                end
            end
        end
        for i = size(x_out,1)-pad(2)+1:size(x_out)
            for j = 1:size(x_out,2)
                for k = 1:size(x_out,3)
                    x_out_1 = size(x_out,1)-pad(2);
                    x_out(i,j,k) = x_out(x_out_1,j,k);
                    writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n',...
                        c_inf.layer,i-1, j-1, k-1, ...
                        c_inf.layer, x_out_1-1, j-1, k-1);
                end
            end
        end       
        for i = 1:size(x_out,1)
            for j = 1:pad(3)
                for k = 1:size(x_out,3)
                    x_out_1 = pad(3)+1;
                    x_out(i,j,k) = x_out(i,x_out_1,k);
                    writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n',...
                        c_inf.layer,i-1, j-1, k-1, ...
                        c_inf.layer, i-1, x_out_1-1, k-1);
                end
            end
        end 
        for i = 1:size(x_out,1)
            for j = size(x_out,2)-pad(4)+1:size(x_out)
                for k = 1:size(x_out,3)
                    x_out_1 = size(x_out,2)-pad(4);
                    x_out(i,j,k) = x_out(i,x_out_1,k);
                    writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n',...
                        c_inf.layer,i-1, j-1, k-1, ...
                        c_inf.layer, i-1, x_out_1-1, k-1);
                end
            end
        end 
    end
    if strcmp(method, 'zero')
        for i = 1:pad(1)
            for j = 1:size(x_out,2)
                for k = 1:size(x_out,3)
                    x_out_1 = pad(1)+1;
                    x_out(i,j,k) = 0;
                    writeC(c_inf, '\tx%i[%i][%i][%i] = 0;\n',...
                        c_inf.layer,i-1, j-1, k-1);
                end
            end
        end
        for i = size(x_out,1)-pad(2)+1:size(x_out)
            for j = 1:size(x_out,2)
                for k = 1:size(x_out,3)
                    x_out_1 = size(x_out,1)-pad(2);
                    x_out(i,j,k) = 0;
                    writeC(c_inf, '\tx%i[%i][%i][%i] = 0;\n',...
                        c_inf.layer,i-1, j-1, k-1);
                end
            end
        end       
        for i = 1:size(x_out,1)
            for j = 1:pad(3)
                for k = 1:size(x_out,3)
                    x_out_1 = pad(3)+1;
                    x_out(i,j,k) = 0;
                    writeC(c_inf, '\tx%i[%i][%i][%i] = 0;\n',...
                        c_inf.layer,i-1, j-1, k-1);
                end
            end
        end 
        for i = 1:size(x_out,1)
            for j = size(x_out,2)-pad(4)+1:size(x_out)
                for k = 1:size(x_out,3)
                    x_out_1 = size(x_out,2)-pad(4);
                    x_out(i,j,k) = 0;
                    writeC(c_inf, '\tx%i[%i][%i][%i] = 0;\n',...
                        c_inf.layer,i-1, j-1, k-1);
                end
            end
        end 
    end
    c_inf.layer = c_inf.layer + 1;

function [x_out,c_inf] = avg_pool(x, p, stride, c_inf)
    z_res = size(x,3);
    x_res = ceil((size(x,1) - p(1) + 1) / stride);
    y_res = ceil((size(x,2) - p(2) + 1) / stride);
    x_out = single(zeros(x_res, y_res, size(x,3)));
    writeC(c_inf, '\tfloat x%i[%i][%i][%i];\n', c_inf.layer, x_res, y_res, z_res);
    for ix = 1:stride:size(x,1) - p(1) + 1
        for jx = 1:stride:size(x,2) - p(2) + 1
            for kx = 1:size(x,3)
                x_out_1 = ceil(ix/stride);
                x_out_2 = ceil(jx/stride);
                x_out(x_out_1,x_out_2,kx) = mean(mean(x(ix:ix+p(1)-1,jx:jx+p(2)-1,kx)));
                writeC(c_inf, '\tx%i[%i][%i][%i] = (0', c_inf.layer, x_out_1-1, x_out_2-1, kx-1);
                for mi = ix:ix+p(1)-1
                    for mj = jx:jx+p(2)-1
                        writeC(c_inf, '+x%i[%i][%i][%i]', c_inf.layer-1,mi-1,mj-1,kx-1);
                    end
                end
                writeC(c_inf, ')/%ff;\n', p(1)*p(2));
            end
        end
    end
    c_inf.layer = c_inf.layer + 1;

function [x_out,c_inf] = max_pool(x, p, stride, c_inf)
    z_res = size(x,3);
    x_res = ceil((size(x,1) - p(1) + 1) / stride);
    y_res = ceil((size(x,2) - p(2) + 1) / stride);
    x_out = single(zeros(x_res, y_res, size(x,3)));
    writeC(c_inf, '\tfloat x%i[%i][%i][%i];\n', c_inf.layer, x_res, y_res, z_res);
    for ix = 1:stride:size(x,1) - p(1) + 1
        for jx = 1:stride:size(x,2) - p(2) + 1
            for kx = 1:size(x,3)
                x_out_1 = ceil(ix/stride);
                x_out_2 = ceil(jx/stride);
                x_out(x_out_1,x_out_2,kx) = max(max(x(ix:ix+p(1)-1,jx:jx+p(2)-1,kx)));
                writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i];\n', ...
                    c_inf.layer,x_out_1-1, x_out_2-1, kx-1, ...
                    c_inf.layer-1, ix-1, jx-1, kx-1);
                for mi = ix:ix+p(1)-1
                    for mj = jx:jx+p(2)-1
                        writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i] > x%i[%i][%i][%i] ? x%i[%i][%i][%i] : x%i[%i][%i][%i];\n',...
                            c_inf.layer,x_out_1-1, x_out_2-1, kx-1, ...
                            c_inf.layer-1, mi-1, mj-1, kx-1, ...
                            c_inf.layer,x_out_1-1, x_out_2-1, kx-1, ...
                            c_inf.layer-1, mi-1, mj-1, kx-1, ...
                            c_inf.layer,x_out_1-1, x_out_2-1, kx-1);
                    end
                end
            end
        end
    end
    c_inf.layer = c_inf.layer + 1;
    
function [x_out, c_inf] = rectifiedLinearUnit(x, c_inf)
    x_out = single(zeros(size(x)));
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            for k = 1:size(x,3)
                x_out(i,j,k) = max(0, x(i,j,k));
                writeC(c_inf, '\tx%i[%i][%i][%i] = x%i[%i][%i][%i] <= 0 ? 0 : x%i[%i][%i][%i];\n',...
                    c_inf.layer-1, i-1, j-1, k-1, ...
                    c_inf.layer-1, i-1, j-1, k-1, ...
                    c_inf.layer-1, i-1, j-1, k-1);
            end
        end
    end
 
function [c_inf] = writeHeader(c_inf)
    c_inf.f = fopen(c_inf.path, 'w');
    c_inf.layer = 1;
    fprintf(c_inf.f, '#include <emmintrin.h>\nint cnn(float x0[%i][%i][1], int *res, float *scores)\n{\n', c_inf.x_dim, c_inf.y_dim);
    
function c_inf = writeFooter(c_inf)
    if c_inf.f ~= -1
        fprintf(c_inf.f, '\treturn 0;\n}\n');
        fclose(c_inf.f);
        c_inf.f = -1;
        assert(system('g++ -msse3 -std=c++11 -g main.c') == 0)
    end
    

function writeC(c_inf, varargin)
    if c_inf.f ~= -1
        fprintf(c_inf.f, varargin{:});
    end
    
