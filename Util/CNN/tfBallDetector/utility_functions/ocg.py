""" Olli's Code Generator (OCG) """

__author__ = "Oliver Urbann"
__copyright__ = "Copyright 2017-2019, Oliver Urbann"
__credits__ = ["Oliver Urbann"]
__license__ = "Commercial, free for educational usage."
__version__ = "1.0.0"
__email__ = "oliver.urbann@tu-dortmund.de"

import random
import os
import numpy as np
import imageio
from keras import backend as K, Model
from keras.layers import Convolution2D, MaxPooling2D, Flatten, Dropout, BatchNormalization, LeakyReLU, InputLayer
from keras.models import load_model
from .stats import printProgressBar
import time

compiler = 'g++ -mssse3 -std=c++11 -g -march=bonnell -DCNN_TEST '
compiler_O3 = 'g++ -O3 -mssse3 -std=c++11 -march=bonnell -DCNN_TEST '
compiler_check ='g++ --version'

#if os.name == 'nt':
#    compiler = 'wsl clang++ -mssse3 -std=c++11 -g -march=bonnell -DCNN_TEST '
#    compiler_O3 = 'wsl clang++ -O3 -mssse3 -std=c++11 -march=bonnell -DCNN_TEST '

disable_warnings = '''
#if defined(__clang__)
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-result"
# pragma clang diagnostic ignored "-Wunused-variable"
# pragma clang diagnostic ignored "-Wconversion"
# pragma clang diagnostic ignored "-Wmissing-braces"
# pragma clang diagnostic ignored "-Wfloat-conversion"
#endif

#if defined(_MSC_VER)
# pragma warning(push, 0)
#endif
'''

restore_warnings = '''
#if defined(__clang__)
# pragma clang diagnostic pop
#endif

#if defined(_MSC_VER)
# pragma warning(pop)
#endif
'''

footer_test = '''
#ifdef CNN_TEST
#include <stdio.h>
#ifdef TIMING
#include <ctime>
#endif
    
int main()
{{
    int i, j, k, res, width, height, max_colour;
    unsigned char byte;
    float x[{x_dim} * {y_dim} * {z_dim}];
    float scores[2];
    FILE *f = fopen("{filename}", "r");
    fscanf (f, "P{version}\\n%d %d\\n%d\\n", &width, &height, &max_colour);
    for (j = 0; j < {x_dim}; j++)
        for (i = 0; i < {y_dim}; i++)
            for (k = 0; k < {z_dim}; k++)
            {{
                fread(&byte, sizeof(unsigned char), 1, f);
                x[j * {x_dim} * {z_dim} + i * {z_dim} + k] = byte / 255.f;
            }}
    fclose(f);
    res = 0;
#ifdef TIMING
    clock_t begin = clock();
	for (i = 0; i < TIMING; i++)  
		cnn_{id}(x, scores);
	 clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	printf("%f %f, %f s ", scores[0], scores[1], elapsed_secs);
#else
    cnn_{id}(x, scores);
#endif
    return scores[1] > scores[0];
}}
#endif
''' + restore_warnings

footer_no_test = '''
#ifdef CNN_TEST
#include <stdio.h>
int main()
{
    printf("Thx for using Olli's Code Generator (OCG)");
    return 0;
}
#endif
''' + restore_warnings

footer_benchmark = '''
#ifdef CNN_TEST
#include <stdio.h>
#include <sys/time.h>

int main()
{{
    int i, j, k, res, width, height, max_colour;
    unsigned char byte;
    float x[{x_dim} * {y_dim} * {z_dim}];
    float scores[2];
    FILE *f = fopen("{filename}", "r");
    fscanf (f, "P{version}\\n%d %d\\n%d\\n", &width, &height, &max_colour);
    for (j = 0; j < {x_dim}; j++)
        for (i = 0; i < {y_dim}; i++)
            for (k = 0; k < {z_dim}; k++)
            {{
                fread(&byte, sizeof(unsigned char), 1, f);
                x[j * {x_dim} * {z_dim} + i * {z_dim} + k] = byte / 255.f;
            }}
    fclose(f);
    res = 0;
    for (int i = 0; i < 1000; i++)
    {{
        gettimeofday(&st,NULL);
        cnn(x, output_tensor);
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
        printf("inference time: %d micro seconds\\n",elapsed);
    }}
    
    // write result to stdout
    for (j = 0; j < x_out_dim; j++)
        for (i = 0; i < y_out_dim; i++)
            for (int k = 0; k < z_out_dim; k++)
                printf("%f\\n", output_tensor[j][i][k]);
    return 0;
}}
#endif
''' + restore_warnings


sse_conv = '''
{indent}w = _mm_set_ps({w3}f, {w2}f, {w1}f, {w0}f);
{indent}x = _mm_load_ps1(&x{prev_layer}[{x_1}][{x_2}][{kw}]);
{indent}y = _mm_mul_ps(w, x);
{indent}x = _mm_load_ps((float*)&x{layer}[{x_out_1}][{x_out_2}][{lw}]);
{indent}x = _mm_add_ps(x, y);
{indent}_mm_store_ps((float*)&x{layer}[{x_out_1}][{x_out_2}][{lw}], x);
'''

sse_conv_2 = '''
{indent}w = _mm_set_ps({w3}f, {w2}f, {w1}f, {w0}f);
{indent}x = _mm_load_ps1(&x{prev_layer}[{x_1}][{x_2}][{kw}]);
{indent}t = _mm_mul_ps(w, x);
{indent}y = _mm_add_ps(y, t);
'''

sse_conv_even = '''
{indent}w = _mm_set_ps({w3}f, {w2}f, {w1}f, {w0}f);
{indent}x = _mm_load_ps1(&x{prev_layer}[{x_1}][{x_2}][{kw}]);
{indent}t = _mm_mul_ps(w, x);
{indent}y = _mm_add_ps(y, t);
'''

sse_conv_odd = '''
{indent}w = _mm_set_ps({w3}f, {w2}f, {w1}f, {w0}f);
{indent}x = _mm_load_ps1(&x{prev_layer}[{x_1}][{x_2}][{kw}]);
{indent}t2 = _mm_mul_ps(w, x);
{indent}y2 = _mm_add_ps(y2, t2);
'''



sse_relu = '''
{indent}x = _mm_load_ps((float*)&x{prev_layer}[{i}][{j}][{k}]);
{indent}x = _mm_max_ps(x, _mm_setzero_ps());
{indent}_mm_store_ps((float*)&x{prev_layer}[{i}][{j}][{k}], x);
'''

sse_leaky = '''
{indent}x = _mm_load_ps((float*)&x{prev_layer}[{i}][{j}][{k}]);
{indent}x = _mm_max_ps(x, _mm_mul_ps(y, x));
{indent}_mm_store_ps((float*)&x{prev_layer}[{i}][{j}][{k}], x);
'''

quantize_loops = '''
{indent}for (int i = 0; i < {H}; i++)
{indent}    for (int j = 0; j < {W}; j++)
{indent}        for (int k = 0; k < {C}; k++)
{indent}            cx_in{layer}[i][j][k] = x{prev_layer}[i][j][k] / {x_scale}f;\n
'''

dequantize_loops = '''
{indent}for (int i = 0; i < {x_res}; i++)
{indent}    for (int j = 0; j < {y_res}; j++)
{indent}        for (int k = 0; k < {z_res}; k++)
{indent}            x{layer}[i][j][k] += cx{layer}[i][j][k] * {w_scale}f * {x_scale}f;\n
'''



quantized_ssse3_conv = '''
{indent}qx = _mm_lddqu_si128((__m128i*)&cx_in{layer}[{x_1}][{x_2}][{kw}]);
{indent}qw = _mm_set_epi8({w15}, {w14}, {w13}, {w12}, {w11}, {w10}, {w9}, {w8}, {w7}, {w6}, {w5}, {w4}, {w3}, {w2}, {w1}, {w0});
{indent}qx = _mm_maddubs_epi16(qx, qw);
{indent}cx{layer}[{x_out_1}][{x_out_2}][{lw}] = _mm_adds_epi16(qx, cx{layer}[{x_out_1}][{x_out_2}][{lw}]);
'''


dequantize_loops_ssse3 = '''
{indent}for (int i = 0; i < {x_res}; i++)
{indent}    for (int j = 0; j < {y_res}; j++)
{indent}        for (int k = 0; k < {z_res}; k++){{
{indent}            qx = cx{layer}[i][j][k];              
{indent}            lo = _mm_srai_epi32(_mm_unpacklo_epi16(qx, qx), 16);
{indent}            hi = _mm_srai_epi32(_mm_unpackhi_epi16(qx, qx), 16);
{indent}            sum1 = _mm_hadd_epi32(hi, lo);
{indent}            sum2 = _mm_hadd_epi32(sum1, sum1);
{indent}	        _mm_store_si128((__m128i*)res, sum2);
{indent}            x{layer}[i][j][k] += (res[0] + res[1]) * {w_scale}f * {x_scale}f;
{indent}        }}
'''


def check_arch(given_arch, min_arch):
    if given_arch == 'ssse3':
        if min_arch == 'general':
            return True
        if min_arch == 'sse3':
            return True
        if min_arch == 'ssse3':
            return True
    if given_arch == 'sse3':
        if min_arch == 'general':
            return True
        if min_arch == 'sse3':
            return True
        if min_arch == 'ssse3':
            return False
    if given_arch == 'general':
        if min_arch == 'general':
            return True
        if min_arch == 'sse3':
            return False
        if min_arch == 'ssse3':
            return False


def compile(c_inf, optimize=False):
    c = compiler_O3 if optimize else compiler
    if os.system(c + c_inf["path"] + " -o " + c_inf["path"][:c_inf["path"].rfind('.')]) != 0:
        print("Error compiling file.")
        exit(-3)

def normalization(x, c_inf, q_inf):
    x_out = np.zeros(x.shape).astype('float32')
    writeC(c_inf, '\tstatic float x{:d}[{:d}][{:d}][{:d}] = {{0}};\n'.format(c_inf["layer"], x.shape[0], x.shape[1], x.shape[2]))
    max_in = q_inf['max_in'][c_inf["layer"] - 1]
    for i in range(size(x, 1)):
        for j in range(size(x, 2)):
            for k in range(size(x, 3)):
                x_out[i, j] = x[i, j] / max_in
                writeC(c_inf, '\tx{:d}[{:d}][{:d}][{:d}] = '
                              'x{:d}[{:d}][{:d}][{:d}] / {:f};\n'.format(c_inf["layer"], i, j, k,
                                                                         c_inf["layer"] - 1, i, j, k, max_in))
    return x_out, c_inf

def softmax(x, c_inf):
    assert(x.shape[2] == 2)  # Sorry, only for depth 2 at the moment
    x_out = np.zeros(x.shape).astype('float32')
    writeC(c_inf, '\tstatic float x{:d}[{:d}][{:d}][{:d}] = {{0}};\n'.format(c_inf["layer"], x.shape[0], x.shape[1], x.shape[2]))

    for i in range(size(x, 1)):
        for j in range(size(x, 2)):
            writeC(c_inf,
                   '\tstatic float max{:d} = x{:d}[{:d}][{:d}][0] > x{:d}[{:d}][{:d}][1] ? x{:d}[{:d}][{:d}][0] : x{:d}[{:d}][{:d}][1];\n'.format(
                       c_inf["layer"],
                       c_inf["layer"] - 1, i, j,
                       c_inf["layer"] - 1, i, j,
                       c_inf["layer"] - 1, i, j,
                       c_inf["layer"] - 1, i, j))
            x_out[i,j] = 2.718281828459045 ** (x - np.max(x))
            writeC(c_inf,
                   '\tx{:d}[{:d}][{:d}][0] = (float)exp(x{:d}[{:d}][{:d}][0] - max{:d});\n'
                   '\tx{:d}[{:d}][{:d}][1] = (float)exp(x{:d}[{:d}][{:d}][1] - max{:d});\n'.format(
                       c_inf["layer"], i, j,
                       c_inf["layer"] - 1, i, j,
                       c_inf["layer"],
                       c_inf["layer"], i, j,
                       c_inf["layer"] - 1, i, j,
                       c_inf["layer"]))

            x_out[i, j] = x_out[i,j] / x_out[i,j].sum()
            writeC(c_inf, '\tstatic float sum{:d};\n'.format(c_inf["layer"]))
            writeC(c_inf, '\tsum{:d} = x{:d}[{:d}][{:d}][0] + x{:d}[{:d}][{:d}][1];\n'.format(
                c_inf["layer"],
                c_inf["layer"], i, j,
                c_inf["layer"], i, j
            ))
            writeC(c_inf, '\tx{:d}[{:d}][{:d}][0] /= sum{:d};\n'
                          '\tx{:d}[{:d}][{:d}][1] /= sum{:d};\n'.format(
                c_inf["layer"], i, j,
                c_inf["layer"],
                c_inf["layer"], i, j,
                c_inf["layer"]))
    return x_out, c_inf


def writeC(c_inf, str):
    if "f" in c_inf and c_inf["f"] is not None:
        c_inf["f"].write(str)


def keras_compile(imdb, model_path, code_path, identifier, unroll_level=0, arch="general",
                  quantize=False, eval=False, replaceSoftMax=False):
    test_im_index = 1000
    im = imdb["images"][test_im_index]
    c_inf = {}
    c_inf["id"] = identifier
    c_inf["path"] = code_path + "/cnn_" + identifier + ".cpp"
    c_inf["x_dim"] = im.shape[0]
    c_inf["y_dim"] = im.shape[1]


    if len(im.shape) > 2:
        c_inf["z_dim"] = im.shape[2]
    else:
        c_inf["z_dim"] = 1
    intermediate_output = []

    if not eval:
        c_inf = writeHeader(c_inf, arch)

    hwc = {'H': size(im,1), 'W': size(im,2), 'C': size(im,3)}
    writeC(c_inf, '\tfloat x0[{H}][{W}][{C}];\n'.format(**hwc))
    if unroll_level > 0:
        writeC(c_inf, '\tfor (int xi = 0; xi < {:d}; xi += 1)\n\t{{\n'.format(size(im,1)))
        _xi = 'xi'
    for xi in range(size(im,1)):
        if unroll_level == 0:
            _xi = xi
        for xj in range(size(im,2)):
            for xk in range(size(im,3)):
                if unroll_level == 0 or xi == 0:
                    writeC(c_inf, '\tx0[{xi}][{xj}][{xk}] = x_in[{xi} * {W} * {C} + {xj} * {C} + {xk}] - {mean}f;\n'.format(
                        xj=xj,xk=xk,mean=imdb["mean"],xi=_xi,**hwc))
    if unroll_level > 0:
        writeC(c_inf, '\t}\n')
    model = load_model(model_path, compile=False)


    max_in = [np.max(imdb["images"])]
    min_in = [np.min(imdb["images"])]
    q_inf = dict()
    q_inf['quantize'] = quantize
    print("Predicting images to find min/max for quantization and others...", end="")
    inp = model.input
    outputs = [layer.output for layer in model.layers]
    functor = K.function([inp, K.learning_phase()], outputs)
    layer_outs = functor([imdb["images"][:10000], 1.])
    for l in layer_outs:
        max_in.append(np.max(l))
        min_in.append(np.min(l))
    print(" finished")
    q_inf['q'] = False
    q_inf['min_in'] = min_in
    q_inf['max_in'] = max_in


    if eval:
        eval_x = imdb["images"]
    else:
        eval_x = imdb["images"][test_im_index].reshape(1, *imdb["images"][test_im_index].shape)

    ok_count = 0
    acc_count = 0
    count = 0
    for x in eval_x:
        _x = np.copy(x)
        skip = False
        c_inf["layer"] = 1
        last_activation = 'none'
        for i, layer in enumerate(model.layers):
            printProgressBar(i, len(model.layers), prefix='Generating code')
            if skip:
                skip = False
                continue
            if type(layer) == Convolution2D:
                w = K.eval(layer.weights[0])
                b = K.eval(layer.bias)
                strides = layer.strides
                padding = layer.padding
                activation = layer.activation

                if i < len(model.layers)-1 and activation.__name__ == 'linear' \
                        and type(model.layers[i+1]) == BatchNormalization:
                    print("Merging Convolution2d -> BatchNormalization")
                    skip = True
                    batch_norm_layer = model.layers[i+1]
                    mean = K.eval(batch_norm_layer.moving_mean)
                    var = K.eval(batch_norm_layer.moving_variance)
                    beta = K.eval(batch_norm_layer.beta)
                    gamma = K.eval(batch_norm_layer.gamma)
                    s = 1.0 / np.sqrt(var + batch_norm_layer.epsilon)
                    a = s
                    if batch_norm_layer.scale:
                        a = a * gamma
                    b_ = -mean * a
                    if batch_norm_layer.center:
                        b_ = b_ + beta

                    w = w * a
                    b = b * a + b_

                    layer = model.layers[i+1]

                _x, c_inf = convolution(_x, w, b, strides, padding, c_inf,
                                        unroll_level, arch, q_inf)

                if activation.__name__ == 'relu':
                    _x, c_inf = rectifiedLinearUnit(_x, c_inf, unroll_level, 0, arch)
                if activation.__name__ == 'softmax':
                    if replaceSoftMax:
                        _x, c_inf = normalization(_x, c_inf, q_inf)
                    else:
                        _x, c_inf = softmax(_x, c_inf)
                last_activation = activation.__name__
            elif type(layer) == MaxPooling2D:
                _x, c_inf = max_pool(_x, layer.pool_size, layer.strides, c_inf, unroll_level, arch)
            elif type(layer) == LeakyReLU:
                _x, c_inf = rectifiedLinearUnit(_x, c_inf, unroll_level, float(layer.alpha), arch)
                last_activation = 'leaky'
            elif type(layer) == Flatten:
                pass
            elif type(layer) == Dropout:
                pass
            elif type(layer) == InputLayer:
                pass
            elif type(layer) == BatchNormalization:
                print("Warning: BatchNormalization not implemented")
                pass
            else:
                print("Unknown layer")
                exit(-1)
            if not quantize and not eval:
                intermediate_layer_model = Model(inputs=model.input,
                                                 outputs=layer.output)
                io = intermediate_layer_model.predict(x.reshape(1, *x.shape))
                intermediate_output.append(io)

                if not np.allclose(_x.reshape(1, *_x.shape), io, atol=0.001):
                    diff = np.abs(_x.reshape(1, *_x.shape) - io)
                    print(str(layer) + ": err, maximum err = ", diff.max())
                else:
                    print(str(layer) + ": ok")
        if eval:
            ok_count += int(np.argmax(_x) == np.argmax(model.predict(x.reshape(1, *x.shape))))
            acc_count += int(np.argmax(_x) == np.argmax(imdb["y"][count]))
            count += 1
            print("\rImg {:d} of {:d}: ok are {:d} ({:f}%), accuracy is {:f}%".format(count,
                                                                                      len(eval_x), ok_count,
                                                                                      ok_count/count*100,
                                                                                      acc_count/count*100),
                  end="")
    printProgressBar(len(model.layers), len(model.layers), prefix='Generating code')

    # last layer with softmax activition indicates a classification problem
    # write scores etc. only in this case
    classification = False
    if last_activation == 'softmax':
        for ix in range(size(_x, 3)):
            writeC(c_inf, '\tscores[{:d}] = x{:d}[{:d}][{:d}][{:d}];\n'.format(
                ix, c_inf["layer"], 0, 0, ix))
        classification = True

    writeFooter(c_inf, _x, classification)

    print("Checking compiler version ...")
    if os.system(compiler_check) == 1:
        print("Compiler not found, not checking code file --> Finished.")
    print("Compiling...")

    compile(c_inf, optimize=True)

    if classification:
        print("Testing...")
        tested = 0
        fail = 0
        for im in np.random.permutation(imdb["images"]):
            if im.shape[2] == 1:
                im = im.reshape(1, *im.shape)
                imageio.imwrite("img.pgm", ((im[0] + imdb["mean"]) * 255).astype("uint8"), "img.pgm")
            else:
                print("Not implemented")
                exit(2)
            if os.name == 'nt':
                res = os.system(c_inf["path"][:c_inf["path"].rfind('.')])
            else:
                res = os.system("./" + c_inf["path"][:c_inf["path"].rfind('.')]) >> 8
            p = np.argmax(model.predict(im))
            if quantize:
                tested += 1
                if res != p:
                    fail += 1
                printProgressBar(tested, len(imdb["images"]),
                                 suffix=", " + str((tested-fail) / tested * 100) + "% ok",
                                 prefix='Evaluating')
            else:
                if res == p:
                    print("Image " + imdb["images"].index(im) + " passed, is " + str(p))
                else:
                    print("Error at image " + imdb["images"].index(im) + " is " + str(p))
                    exit(-2)
    else:
        print("Running...")
        im.tofile("img.bin")
        expected_result = model.predict(im.reshape(1, *im.shape))

        output = os.popen(c_inf["path"][:-2]).read()

        values = [float(o) for o in output.split("\n") if o and not o.startswith("inference")]
        c_result = np.array(values).reshape(expected_result.shape)
        if not np.allclose(expected_result, c_result, atol=0.001):
            diff = np.abs(expected_result - c_result)
            print("Error in result of c code, maximum err = ", diff.max())
        else:
            print("Result of c code is correct")

        times = [float(o.split()[2]) for o in output.split("\n") if o.startswith("inference")]
        times = np.array(times)
        print("runs: {}, total: {} micro seconds, mean: {} micro seconds".format(times.size, times.sum(), times.mean()))



def size(x, i=1):
    return x.shape[i-1]


def writeHeader(c_inf, arch):
    c_inf["f"] = open(c_inf["path"], 'w')
    if check_arch(arch, 'sse3'):
        c_inf["f"].write(disable_warnings)
        c_inf["f"].write('#include <emmintrin.h>\n')
        c_inf["f"].write('#include <pmmintrin.h>\n')
        c_inf["f"].write('#include <tmmintrin.h>\n')
        c_inf["f"].write('#include <immintrin.h>\n')
    c_inf["f"].write('#include <math.h>\nvoid cnn_{:s}(float *x_in, float *scores)\n'.format(c_inf["id"]) + '{\n')
    if check_arch(arch, 'sse3'):
        c_inf["f"].write('\t__m128 w, x, y, y2, t, t2;\n'
                         'unsigned char buf alignas(16) [16];\n'
                         '__m128i qw, qx, qt1, qt2, lo, hi, sum1, sum2, sum3;\n'
                         'int res alignas(16) [4];\n')

    return c_inf


def writeFooter(c_inf, _x, classification):
    if c_inf["f"] is not None:
        c_inf["f"].write('\treturn;\n}\n')
        if classification:
            c_inf["filename"] = "img.pgm"
            c_inf["version"] = "5"
            c_inf["f"].write(footer_test.format(**c_inf))
        else:
            print("Unimplemented")
            exit(2)
            c_inf["f"].write(footer_benchmark.format(x_dim=c_inf['x_dim'], y_dim=c_inf['y_dim'], z_dim=c_inf['z_dim'],
                                                     *_x.shape))
            with open(c_inf["path"], 'r') as file:
                data = file.read()
            data = data.replace("int *res, float *scores", ' float output_tensor[{:d}][{:d}][{:d}]'.format(
                size(_x, 1),
                size(_x, 2),
                size(_x, 3)))
            data = data.replace('x{:d}['.format(c_inf["layer"] - 1), 'output_tensor[')
            with open(c_inf["path"], 'w') as file:
                file.write(data)
        c_inf["f"].close()
        c_inf["f"] = None
    return c_inf

def quantize_scale(min, max, type):
    if abs(min) > abs(max):
        v = abs(min)
    else:
        v = abs(max)
    if type == 'int8':
        return v / 127
    elif type == 'uint8':
        return v / 256


def convolution(x, w, b, stride, pad, c_inf, unroll_level, arch, q_inf):
    assert x.shape[2] == w.shape[2]
    assert w.shape[3] == b.shape[0]
    H, W, C_IN = x.shape
    KH, KW, _, C_OUT = w.shape
    SH, SW = stride

    if pad == 'valid':
        H_OUT = int(np.ceil((H - KH + 1) / SH))
        W_OUT = int(np.ceil((W - KW + 1) / SW))
        pad_top = pad_bottom = pad_left = pad_right = 0
    elif pad == 'same':
        H_OUT = int(np.ceil(float(H) / float(SH)))
        W_OUT = int(np.ceil(float(W) / float(SW)))
        pad_along_height = max((H_OUT - 1) * SH + KH - H, 0)
        pad_along_width = max((W_OUT - 1) * SW + KW - W, 0)
        pad_top = int(pad_along_height // 2)
        pad_bottom = int(pad_along_height - pad_top)
        pad_left = int(pad_along_width // 2)
        pad_right = int(pad_along_width - pad_left)
    else:
        print("Unknown padding type")
        exit(-4)
    x_out = np.zeros((H_OUT, W_OUT, C_OUT)).astype('float32')
    str_data = {'prev_layer': c_inf["layer"] - 1, 'x_1': 'x_1', 'x_2': 'x_2',
                'x_out_1': 'x_out_1', 'x_out_2': 'x_out_2', 'layer': c_inf["layer"],
                'lw': 'lw', 'indent': '\t', 'x_res': H_OUT, 'y_res': W_OUT, 'z_res': C_OUT,
                'stride0': stride[0], 'stride1': stride[1], 'i': 'i', 'j': 'j',
                'pt': pad_top, 'pb': pad_bottom, 'pl': pad_left, 'pr': pad_right}

    writeC(c_inf, '{indent}static float x{layer} alignas(16) [{x_res}][{y_res}][{z_res}] = {{0}};\n'.format(**str_data))

    # Positive x is required for quantization as in SSSE3 one of the values must be quantized to uint8
    if q_inf['quantize'] and np.min(x) >= 0 and \
            ((C_IN % 16 == 0 and check_arch(arch, 'ssse3')) or
             arch == 'general'):
        quantize = True
    else:
        quantize = False

    if quantize:
        min_in = q_inf['min_in'][c_inf["layer"]-1]
        max_in = q_inf['max_in'][c_inf["layer"]-1]
        x_scale = quantize_scale(min_in, max_in, 'uint8')
        w_scale = quantize_scale(np.min(w), np.max(w), 'int8')
        str_data['x_scale'] = x_scale
        str_data['w_scale'] = w_scale
        w = (w / w_scale).astype('int8')
        x = (x / x_scale).astype('uint8')

        writeC(c_inf,
               '{indent}static __m128i cx{layer} alignas(16) [{x_res}][{y_res}][{z_res}];\n'.format(**str_data))
        writeC(c_inf, '{indent}static unsigned char cx_in{layer} alignas(16) '
                      '[{H}][{W}][{C}];\n'.format(H=H, W=W, C=C_IN, **str_data))
        if unroll_level == 0:
            for i in range(H):
                str_data['i'] = i
                for j in range(W):
                    str_data['j'] = j
                    for k in range(C_IN):
                        str_data['k'] = k
                        writeC(c_inf, '{indent}cx_in{layer}[{i}][{j}][{k}] = x{prev_layer}[{i}][{j}][{k}] / {x_scale}f;\n'
                               .format(**str_data))
        else:
            writeC(c_inf, quantize_loops.format(H=H, W=W, C=C_IN, **str_data))


    if unroll_level > 0:
        writeC(c_inf, '{indent}for (int i = 0; i < {:d}; i += 1)\n{indent}{{\n'.format(H_OUT, **str_data))
        str_data['indent'] = '\t\t'
    for i in range(H_OUT):
        if unroll_level > 1 and i == 0:
            writeC(c_inf, '{indent}for (int j = 0; j < {:d}; j += 1)\n{indent}{{\n'.format(W_OUT, **str_data))
            str_data['indent'] = '\t\t\t'
        if unroll_level == 0:
            str_data['i'] = i
        for j in range(W_OUT):
            if unroll_level < 2:
                str_data['j'] = j
            for kw in range(C_OUT):
                str_data['kw'] = kw
                str_data['b'] = b[kw]
                x_out[i, j, kw] = b[kw]
                if unroll_level == 0 or (unroll_level == 1 and i == 0) or (unroll_level == 2 and i == 0 and j == 0):
                    writeC(c_inf, '{indent}x{layer}[{i}][{j}][{kw}] = {b}f;\n'.format(**str_data))
                    if quantize:
                        writeC(c_inf, '{indent}cx{layer}[{i}][{j}][{kw}] = _mm_setzero_si128();\n'.format(**str_data))
        if unroll_level > 1 and i == 0:
            str_data['indent'] = '\t\t'
            writeC(c_inf, '{indent}}}\n'.format(**str_data))
    if unroll_level > 0:
        str_data['indent'] = '\t'
        writeC(c_inf, '{indent}}}\n'.format(**str_data))


    if unroll_level > 0:
        writeC(c_inf, '{indent}for (int ix = -{pt}; ix < {:d}; ix += {stride0})\n{indent}{{\n'.format(
            H - KH + 1 + pad_bottom, **str_data))
        str_data['indent'] = '\t\t'
        writeC(c_inf, '{indent}int x_1, x_out_1;\n'.format(**str_data))

    for ix in range(-pad_top, H - KH + 1 + pad_bottom, SH):
        assert (ix + pad_top) % SH == 0
        x_out_1 = (ix + pad_top) // SH

        if unroll_level > 0 and ix == -pad_top:
            writeC(c_inf, '{indent}x_out_1 = (ix + {pt}) / {stride0};\n'.format(**str_data))
        else:
            str_data['x_out_1'] = x_out_1

        if unroll_level > 1 and ix == -pad_top:
            writeC(c_inf, '{indent}for (int jx = -{pl}; jx < {:d}; jx += {stride1})\n{indent}{{\n'.format(
                W - KW + 1 + pad_right, **str_data))
            str_data['indent'] = '\t\t\t'
            writeC(c_inf, '{indent}int x_2, x_out_2;\n'.format(**str_data))

        for jx in range(-pad_left, W - KW + 1 + pad_right, SW):
            assert (jx + pad_left) % SW == 0
            x_out_2 = (jx + pad_left) // SW

            if unroll_level > 1 and -pad_left == jx and ix == -pad_top:
                writeC(c_inf, '{indent}x_out_2 = (jx + {pl}) / {stride1};\n'.format(**str_data))
            else:
                str_data['x_out_2'] = x_out_2

            for iw in range(KH):
                x_1 = ix + iw
                if (ix == -pad_top and unroll_level == 1) or (unroll_level == 2 and jx == -pad_left and ix == -pad_top):
                    writeC(c_inf, '{indent}x_1 = ix + {:d};\n'.format(iw,  **str_data))
                    if pad == 'same':
                        writeC(c_inf, '{indent}if (x_1 >= 0 && x_1 < {:d})\n{indent}{{\n'.format(H, **str_data))
                        str_data['indent'] += '\t'
                else:
                    str_data['x_1'] = x_1
                for jw in range(KW):
                    x_2 = jx + jw
                    if unroll_level > 1 and jx == -pad_left and ix == -pad_top:
                        writeC(c_inf, '{indent}x_2 = jx + {:d};\n'.format(jw, **str_data))
                        if pad == 'same':
                            writeC(c_inf, '{indent}if (x_2 >= 0 && x_2 < {:d})\n{indent}{{\n'.format(W, **str_data))
                            str_data['indent'] += '\t'
                    else:
                        str_data['x_2'] = x_2
                    for kw in range(C_IN):
                        str_data['kw'] = kw
                        for lw in range(0, C_OUT):
                            str_data['lw'] = lw
                            if quantize: #  Make sure that this is only true if a case below is True and handled
                                if check_arch(arch, 'ssse3') and kw % 16 == 0:
                                    for i in range(16):
                                        str_data['w' + str(i)] = w[iw, jw, kw + i, lw]
                                    if x_1 >= 0 and x_1 < H:
                                        if x_2 >= 0 and x_2 < W:
                                            for _k in range(16):
                                                int_x = w[iw, jw, kw + _k, lw] * x[x_1, x_2, kw + _k].astype('int16')
                                                x_out[x_out_1, x_out_2, lw] += int_x * w_scale * x_scale
                                    if (unroll_level == 0 and x_1 >= 0 and x_1 < H and x_2 >= 0 and x_2 < W) \
                                            or (ix == -pad_top and unroll_level == 1 and x_2 >= 0 and x_2 < W) or \
                                            (unroll_level == 2 and jx == -pad_left and ix == -pad_top):
                                        writeC(c_inf, quantized_ssse3_conv.format(**str_data))
                                elif arch == 'general':
                                    str_data['w0'] = w[iw, jw, kw, lw]
                                    if x_1 >= 0 and x_1 < H:
                                        if x_2 >= 0 and x_2 < W:
                                            int_x = w[iw, jw, kw, lw] * x[x_1, x_2, kw].astype('int16')
                                            x_out[x_out_1, x_out_2, lw] += int_x * w_scale * x_scale
                                    if (unroll_level == 0 and x_1 >= 0 and x_1 < H and x_2 >= 0 and x_2 < W) \
                                            or (ix == -pad_top and unroll_level == 1 and x_2 >= 0 and x_2 < W) or \
                                            (unroll_level == 2 and jx == -pad_left and ix == -pad_top):
                                        writeC(c_inf,
                                               '{indent}cx{layer}[{x_out_1}][{x_out_2}][{lw}] += '
                                               '{w0} * cx_in{layer}[{x_1}][{x_2}][{kw}];\n'.format(**str_data))
                            else:
                                if check_arch(arch, 'sse3') and lw % 4 == 0 and C_OUT % 4 == 0:
                                    str_data['w0'] = w[iw, jw, kw, lw]
                                    str_data['w1'] = w[iw, jw, kw, lw + 1]
                                    str_data['w2'] = w[iw, jw, kw, lw + 2]
                                    str_data['w3'] = w[iw, jw, kw, lw + 3]
                                    if x_1 >= 0 and x_1 < H:
                                        if x_2 >= 0 and x_2 < W:
                                            x_out[x_out_1, x_out_2, lw] = (
                                                    x_out[x_out_1, x_out_2, lw] + w[iw, jw, kw, lw] * x[x_1, x_2, kw]).astype(
                                                'float32')
                                            x_out[x_out_1, x_out_2, lw + 1] = (
                                                    x_out[x_out_1, x_out_2, lw + 1] + w[iw, jw, kw, lw + 1] * x[
                                                x_1, x_2, kw]).astype('float32')
                                            x_out[x_out_1, x_out_2, lw + 2] = (
                                                    x_out[x_out_1, x_out_2, lw + 2] + w[iw, jw, kw, lw + 2] * x[
                                                x_1, x_2, kw]).astype('float32')
                                            x_out[x_out_1, x_out_2, lw + 3] = (
                                                    x_out[x_out_1, x_out_2, lw + 3] + w[iw, jw, kw, lw + 3] * x[
                                                x_1, x_2, kw]).astype('float32')
                                    if (unroll_level == 0 and x_1 >= 0 and x_1 < H and x_2 >= 0 and x_2 < W) \
                                        or (ix == -pad_top and unroll_level == 1 and x_2 >= 0 and x_2 < W) or \
                                            (unroll_level == 2 and jx == -pad_left and ix == -pad_top):
                                        writeC(c_inf, sse_conv.format(**str_data))
                                elif arch == 'general' or (check_arch(arch, 'sse3') and C_OUT % 4 != 0):
                                    str_data['w0'] = w[iw, jw, kw, lw]
                                    if x_1 >= 0 and x_1 < H:
                                        if x_2 >= 0 and x_2 < W:
                                            x_out[x_out_1, x_out_2, lw] = \
                                                (x_out[x_out_1, x_out_2, lw] +
                                                 w[iw, jw, kw, lw] * x[x_1, x_2, kw]).astype('float32')
                                    if (unroll_level == 0 and x_1 >= 0 and x_1 < H and x_2 >= 0 and x_2 < W) \
                                        or (ix == -pad_top and unroll_level == 1 and x_2 >= 0 and x_2 < W) or \
                                            (unroll_level == 2 and jx == -pad_left and ix == -pad_top):
                                        writeC(c_inf,
                                               '{indent}x{layer}[{x_out_1}][{x_out_2}][{lw}] += '
                                               '{w0}f * x{prev_layer}[{x_1}][{x_2}][{kw}];\n'.format(**str_data))

                    if unroll_level > 1 and jx == -pad_left and ix == -pad_top and pad == 'same':
                        str_data['indent'] = str_data['indent'][:-1]
                        writeC(c_inf, '{indent}}}\n'.format(**str_data))
                if ((ix == -pad_top and unroll_level == 1) or \
                        (unroll_level == 2 and jx == -pad_left and ix == -pad_top)) and pad == 'same':
                    str_data['indent'] = str_data['indent'][:-1]
                    writeC(c_inf, '{indent}}}\n'.format(**str_data))
        if unroll_level > 1 and ix == -pad_top:
            str_data['indent'] = '\t\t'
            writeC(c_inf, '{indent}}}\n'.format(**str_data))

    if unroll_level > 0:
        str_data['indent'] = '\t'
        writeC(c_inf, '{indent}}}\n'.format(**str_data))

    if quantize:
        writeC(c_inf, dequantize_loops_ssse3.format(**str_data))

    c_inf["layer"] = c_inf["layer"] + 1
    return x_out, c_inf

def rectifiedLinearUnit(x, c_inf, unroll_level, alpha=0.0, arch='general'):
    x_out = np.zeros(x.shape).astype('float32')
    str_data = {'prev_layer': c_inf["layer"] - 1, 'i': 'i', 'j': 'j', 'k': 'k',
                'below_str': '0', 'indent': '\t'}
    if check_arch(arch, 'sse3') and not alpha == 0.0:
        writeC(c_inf, "{indent}y = _mm_set_ps1({alpha}f);\n".format(alpha=alpha, **str_data))
    if unroll_level > 0:
        writeC(c_inf, '\tfor (int i = 0; i < {:d}; i += 1)\n\t{{\n'.format(size(x, 1)))
        str_data['indent'] += '\t'
    for i in range(size(x, 1)):
        if unroll_level > 1 and i == 0:
            writeC(c_inf, '\t\tfor (int j = 0; j < {:d}; j += 1)\n\t\t{{\n'.format(size(x, 2)))
            str_data['indent'] += '\t'
        elif unroll_level == 0:
            str_data['i'] = i
        for j in range(size(x, 2)):
            if unroll_level < 2:
                str_data['j'] = j
            for k in range(size(x, 3)):
                if check_arch(arch, 'sse3'):
                    str_data['k'] = k
                    if x[i, j, k] < 0:
                        x_out[i, j, k] = alpha * x[i, j, k]
                    else:
                        x_out[i, j, k] = x[i, j, k]
                    if unroll_level == 0 or (unroll_level == 1 and i == 0) or (unroll_level == 2 and i == 0 and j == 0):
                        if k % 4 == 0:
                            if alpha == 0.0:
                                writeC(c_inf, sse_relu.format(**str_data))
                            else:
                                writeC(c_inf, sse_leaky.format(**str_data))
                else:
                    str_data['k'] = k
                    if alpha != 0.0:
                        str_data['below_str'] = '{:f}f * x{prev_layer}[{i}][{j}][{k}]'.format(alpha, **str_data)
                    if x[i, j, k] < 0:
                        x_out[i, j, k] = alpha * x[i, j, k]
                    else:
                        x_out[i, j, k] = x[i, j, k]
                    if unroll_level == 0 or (unroll_level == 1 and i == 0) or (unroll_level == 2 and i == 0 and j == 0):
                        writeC(c_inf,
                               '{indent}x{prev_layer}[{i}][{j}][{k}] = x{prev_layer}[{i}][{j}][{k}] < 0 ? '
                               '{below_str} : x{prev_layer}[{i}][{j}][{k}];\n'.format(**str_data))
        if unroll_level > 1 and i == 0:
            writeC(c_inf, '\t\t}\n')
    if unroll_level > 0:
        writeC(c_inf, '\t}\n')
    return x_out, c_inf


def max_pool(x, p, stride, c_inf, unroll_level, arch='general'):
    z_res = size(x, 3)
    x_res = int(np.ceil((size(x, 1) - p[0] + 1) / stride[0]))
    y_res = int(np.ceil((size(x, 2) - p[1] + 1) / stride[1]))
    x_out = np.zeros((x_res, y_res, size(x, 3))).astype('float32')
    str_data = {'prev_layer': c_inf["layer"] - 1, 'ix': 'ix', 'jx': 'jx', 'kx': 'kx', 'indent': '\t',
                'layer': c_inf["layer"], 'mi': 'mi', 'mj': 'mj', 'x_out_1': 'x_out_1', 'x_out_2': 'x_out_2',
                'stride0': stride[0], 'p': p[0], 'x_res': x_res, 'y_res': y_res, 'z_res': z_res, 'stride1': stride[1]}
    writeC(c_inf, '\tstatic float x{layer}[{x_res}][{y_res}][{z_res}] = {{0}};\n'.format(**str_data))
    if unroll_level > 0:
        writeC(c_inf, '\tfor (int ix = 0; ix < {:d}; ix += {stride0})\n\t{{\n'.format(size(x, 1) - p[0] + 1, **str_data))
        str_data['indent'] = '\t\t'
        writeC(c_inf, '{indent}int x_1, x_out_1;\n'.format(**str_data))
    for ix in range(0, size(x, 1) - p[0] + 1, stride[0]):
        x_out_1 = ix // stride[0] # ix is a multiple of stride[0], so integer division is fine
        if unroll_level > 0 and ix == 0:
            writeC(c_inf, '{indent}x_out_1 = ix / {stride0};\n'.format(**str_data))
            if unroll_level == 2:
                writeC(c_inf, '\tfor (int jx = 0; jx < {:d}; jx += {stride1})\n\t{{\n'.format(size(x, 2) - p[1] + 1,
                                                                                              **str_data))
                str_data['indent'] = '\t\t'
                writeC(c_inf, '{indent}int x_2, x_out_2;\n'.format(**str_data))
        elif unroll_level == 0:
            str_data['x_out_1'] = x_out_1
            str_data['ix'] = ix
        for jx in range(0, size(x, 2) - p[1] + 1, stride[1]):
            x_out_2 = jx // stride[1] # jx is a multiple of stride[1], so integer division is fine
            if unroll_level == 2 and ix == 0 and jx == 0:
                writeC(c_inf, '{indent}x_out_2 = jx / {stride1};\n'.format(**str_data))
            if unroll_level < 2:
                str_data['x_out_2'] = x_out_2
                str_data['jx'] = jx
            for kx in range(size(x, 3)):
                str_data['kx'] = kx
                x_out[x_out_1, x_out_2, kx] = np.max(np.max(x[ix:ix + p[0], jx:jx + p[1], kx]))
                if unroll_level == 0 or (unroll_level == 1 and ix == 0) or (unroll_level == 2 and ix == 0 and jx == 0):
                    if check_arch(arch, 'sse3'):
                        if kx % 4 == 0:
                            writeC(c_inf, "{indent}x = _mm_load_ps((float*)&x{prev_layer}[{ix}][{jx}][{kx}]);\n".format(**str_data))
                    else:
                        writeC(c_inf, '{indent}x{layer}[{x_out_1}][{x_out_2}][{kx}] = x{prev_layer}[{ix}][{jx}][{kx}];\n'.format(**str_data))
                for mi in range(0, p[0]):
                    if unroll_level == 0:
                        str_data['mi'] = ix + mi
                    else:
                        str_data['mi'] = 'ix + ' + str(mi)
                    for mj in range(jx, jx + p[1]):
                        if unroll_level < 2:
                            str_data['mj'] = mj
                        else:
                            str_data['mj'] = 'jx + ' + str(mj)
                        if unroll_level == 0 or (unroll_level == 1 and ix == 0) or (unroll_level == 2 and ix == 0 and jx == 0):
                            if check_arch(arch, 'sse3'):
                                if kx % 4 == 0:
                                    writeC(c_inf, "{indent}y = _mm_load_ps((float*)&x{prev_layer}[{mi}][{mj}][{kx}]);\n".format(**str_data))
                                    writeC(c_inf, "{indent}x = _mm_max_ps(x, y);\n".format(**str_data))
                            else:
                                writeC(c_inf,
                                       '{indent}x{layer}[{x_out_1}][{x_out_2}][{kx}] = '
                                       'x{prev_layer}[{mi}][{mj}][{kx}] > x{layer}[{x_out_1}][{x_out_2}][{kx}] ?'
                                       ' x{prev_layer}[{mi}][{mj}][{kx}] : x{layer}[{x_out_1}][{x_out_2}][{kx}];\n'.format(**str_data))
                    if unroll_level == 0 or (unroll_level == 1 and ix == 0) or (unroll_level == 2 and ix == 0 and jx == 0):
                        if check_arch(arch, 'sse3') and kx % 4 == 0:
                            writeC(c_inf, "{indent}_mm_store_ps((float*)&x{layer}[{x_out_1}][{x_out_2}][{kx}], x);\n".format(**str_data))
        if unroll_level > 1 and ix == 0:
            writeC(c_inf, '\t\t}\n')
    c_inf["layer"] = c_inf["layer"] + 1
    if unroll_level > 0:
        writeC(c_inf, '\t}\n')
    return x_out, c_inf
