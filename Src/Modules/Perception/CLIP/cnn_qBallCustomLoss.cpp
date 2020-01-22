
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
#include <emmintrin.h>
#include <pmmintrin.h>
#include <tmmintrin.h>
#include <immintrin.h>
#include <math.h>
void cnn_qBallCustomLoss(float *x_in, float *scores)
{
	__m128 w, x, y, y2, t, t2;
unsigned char buf alignas(16) [16];
__m128i qw, qx, qt1, qt2, lo, hi, sum1, sum2, sum3;
int res alignas(16) [4];
	float x0[16][16][1];
	for (int xi = 0; xi < 16; xi += 1)
	{
	x0[xi][0][0] = x_in[xi * 16 * 1 + 0 * 1 + 0] - 0.585318155956149f;
	x0[xi][1][0] = x_in[xi * 16 * 1 + 1 * 1 + 0] - 0.585318155956149f;
	x0[xi][2][0] = x_in[xi * 16 * 1 + 2 * 1 + 0] - 0.585318155956149f;
	x0[xi][3][0] = x_in[xi * 16 * 1 + 3 * 1 + 0] - 0.585318155956149f;
	x0[xi][4][0] = x_in[xi * 16 * 1 + 4 * 1 + 0] - 0.585318155956149f;
	x0[xi][5][0] = x_in[xi * 16 * 1 + 5 * 1 + 0] - 0.585318155956149f;
	x0[xi][6][0] = x_in[xi * 16 * 1 + 6 * 1 + 0] - 0.585318155956149f;
	x0[xi][7][0] = x_in[xi * 16 * 1 + 7 * 1 + 0] - 0.585318155956149f;
	x0[xi][8][0] = x_in[xi * 16 * 1 + 8 * 1 + 0] - 0.585318155956149f;
	x0[xi][9][0] = x_in[xi * 16 * 1 + 9 * 1 + 0] - 0.585318155956149f;
	x0[xi][10][0] = x_in[xi * 16 * 1 + 10 * 1 + 0] - 0.585318155956149f;
	x0[xi][11][0] = x_in[xi * 16 * 1 + 11 * 1 + 0] - 0.585318155956149f;
	x0[xi][12][0] = x_in[xi * 16 * 1 + 12 * 1 + 0] - 0.585318155956149f;
	x0[xi][13][0] = x_in[xi * 16 * 1 + 13 * 1 + 0] - 0.585318155956149f;
	x0[xi][14][0] = x_in[xi * 16 * 1 + 14 * 1 + 0] - 0.585318155956149f;
	x0[xi][15][0] = x_in[xi * 16 * 1 + 15 * 1 + 0] - 0.585318155956149f;
	}
	static float x1 alignas(16) [16][16][16] = {0};
	for (int i = 0; i < 16; i += 1)
	{
		for (int j = 0; j < 16; j += 1)
		{
			x1[i][j][0] = -0.05212542042136192f;
			x1[i][j][1] = 0.0689864307641983f;
			x1[i][j][2] = -0.0569847971200943f;
			x1[i][j][3] = -0.3792406916618347f;
			x1[i][j][4] = 0.03923352062702179f;
			x1[i][j][5] = -0.1398451179265976f;
			x1[i][j][6] = 0.013174201361835003f;
			x1[i][j][7] = -0.4639430046081543f;
			x1[i][j][8] = -0.0093160979449749f;
			x1[i][j][9] = -0.06962127983570099f;
			x1[i][j][10] = -0.3958790600299835f;
			x1[i][j][11] = 0.05102357268333435f;
			x1[i][j][12] = -0.1446855217218399f;
			x1[i][j][13] = -0.2059023529291153f;
			x1[i][j][14] = -0.07417980581521988f;
			x1[i][j][15] = 0.0007251549977809191f;
		}
	}
	for (int ix = -1; ix < 15; ix += 1)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 1) / 1;
		for (int jx = -1; jx < 15; jx += 1)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 1) / 1;
			x_1 = ix + 0;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.25604695081710815f, -0.07586997747421265f, -0.3307625651359558f, -0.10993970930576324f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.2282310277223587f, 0.011922602541744709f, 0.7153604626655579f, 0.018883610144257545f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-1.0206941366195679f, 0.0059207649901509285f, -0.28321340680122375f, 0.11340763419866562f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.3668573796749115f, -0.44160762429237366f, -0.42137157917022705f, 0.8603530526161194f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.5583449602127075f, 0.015760986134409904f, -0.6670867204666138f, 0.6400661468505859f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.05058325454592705f, -0.3259808421134949f, -0.07776901125907898f, -0.47452637553215027f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.17660285532474518f, -0.07406612485647202f, -0.10245714336633682f, 0.10153583437204361f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.1278519630432129f, 0.4370659589767456f, -0.03331848978996277f, 0.775290310382843f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.34019845724105835f, -0.10510940849781036f, -0.33148691058158875f, -0.5163178443908691f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.3212243318557739f, -0.4661332368850708f, -0.27556973695755005f, -0.4870784282684326f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.210001140832901f, -0.186275452375412f, 0.05488993972539902f, 0.036019425839185715f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.360628604888916f, 0.8628386855125427f, -0.005719422362744808f, 0.08699654042720795f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
			}
			x_1 = ix + 1;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.5327760577201843f, -0.36364859342575073f, -0.3803671896457672f, -0.6743965744972229f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.2364162653684616f, 0.3605804145336151f, 0.4375099241733551f, 0.3449864089488983f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.7177078723907471f, -0.4575618803501129f, 0.21914367377758026f, -0.221271812915802f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.08836818486452103f, -0.41615423560142517f, -0.35599660873413086f, -0.9201704263687134f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.04282166436314583f, -0.21029403805732727f, -0.27494630217552185f, 0.8185247778892517f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.539094865322113f, 0.4642581343650818f, -1.0149073600769043f, -0.3443323075771332f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.49230045080184937f, -0.29150646924972534f, 1.0241546630859375f, 0.028274208307266235f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.7880300879478455f, -0.3453372120857239f, -0.45133259892463684f, -0.6745136976242065f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.528890073299408f, 0.16699548065662384f, 0.15059508383274078f, 0.036814942955970764f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.517251193523407f, -0.2216130793094635f, 0.6517369747161865f, -0.5611302256584167f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.0732472613453865f, -0.35821813344955444f, 0.6266728043556213f, -0.2282070517539978f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.5766895413398743f, 0.8357973098754883f, -0.24002666771411896f, 0.11109372973442078f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
			}
			x_1 = ix + 2;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.628407895565033f, -0.666607677936554f, -0.032789457589387894f, -0.5105113983154297f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.4768688678741455f, -0.3589695394039154f, -0.08772796392440796f, 0.9507489800453186f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.27329525351524353f, 0.39383018016815186f, 0.10573118180036545f, -0.21676617860794067f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.21138769388198853f, -0.5013716816902161f, 0.5016289949417114f, -0.004444543272256851f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.41673529148101807f, 0.18804699182510376f, 0.49947303533554077f, 0.4771065413951874f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.19185207784175873f, 0.18127834796905518f, -0.5652155876159668f, 0.423721581697464f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.02965608239173889f, -0.022895589470863342f, -0.678943395614624f, -0.05910320207476616f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.08241938799619675f, -0.5638787150382996f, 0.5165108442306519f, 0.020881401374936104f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.4295675754547119f, 1.0520744323730469f, 1.2954539060592651f, -0.13862349092960358f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.11458433419466019f, 0.2196265608072281f, 0.37010765075683594f, -0.018640100955963135f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.20652489364147186f, -0.22818799316883087f, -0.8668228387832642f, -0.2606215476989746f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.9349665641784668f, 0.34605059027671814f, 0.5536379814147949f, -0.0175959300249815f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][12], x);
				}
			}
		}
	}
	for (int i = 0; i < 16; i += 1)
	{
		for (int j = 0; j < 16; j += 1)
		{

			x = _mm_load_ps((float*)&x1[i][j][0]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][0], x);

			x = _mm_load_ps((float*)&x1[i][j][4]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][4], x);

			x = _mm_load_ps((float*)&x1[i][j][8]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][8], x);

			x = _mm_load_ps((float*)&x1[i][j][12]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][12], x);
		}
	}
	static float x2[8][8][16] = {0};
	for (int ix = 0; ix < 15; ix += 2)
	{
		int x_1, x_out_1;
		x_out_1 = ix / 2;
	for (int jx = 0; jx < 15; jx += 2)
	{
		int x_2, x_out_2;
		x_out_2 = jx / 2;
		x = _mm_load_ps((float*)&x1[ix][jx][0]);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][0], x);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][0], x);
		x = _mm_load_ps((float*)&x1[ix][jx][4]);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][4], x);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][4], x);
		x = _mm_load_ps((float*)&x1[ix][jx][8]);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][8], x);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][8], x);
		x = _mm_load_ps((float*)&x1[ix][jx][12]);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 0][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][12], x);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x1[ix + 1][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x2[x_out_1][x_out_2][12], x);
		}
	}
	static float x3 alignas(16) [8][8][32] = {0};
	static __m128i cx3 alignas(16) [8][8][32];
	static unsigned char cx_in3 alignas(16) [8][8][16];

	for (int i = 0; i < 8; i++)
	    for (int j = 0; j < 8; j++)
	        for (int k = 0; k < 16; k++)
	            cx_in3[i][j][k] = x2[i][j][k] / 0.008748259395360947f;

	for (int i = 0; i < 8; i += 1)
	{
		for (int j = 0; j < 8; j += 1)
		{
			x3[i][j][0] = -0.04680550843477249f;
			cx3[i][j][0] = _mm_setzero_si128();
			x3[i][j][1] = -0.6932594776153564f;
			cx3[i][j][1] = _mm_setzero_si128();
			x3[i][j][2] = -0.05195837467908859f;
			cx3[i][j][2] = _mm_setzero_si128();
			x3[i][j][3] = -0.031572967767715454f;
			cx3[i][j][3] = _mm_setzero_si128();
			x3[i][j][4] = -0.0881546214222908f;
			cx3[i][j][4] = _mm_setzero_si128();
			x3[i][j][5] = -0.11747313290834427f;
			cx3[i][j][5] = _mm_setzero_si128();
			x3[i][j][6] = -0.23983299732208252f;
			cx3[i][j][6] = _mm_setzero_si128();
			x3[i][j][7] = -0.5322098731994629f;
			cx3[i][j][7] = _mm_setzero_si128();
			x3[i][j][8] = -0.06518011540174484f;
			cx3[i][j][8] = _mm_setzero_si128();
			x3[i][j][9] = 0.20786894857883453f;
			cx3[i][j][9] = _mm_setzero_si128();
			x3[i][j][10] = 0.32214707136154175f;
			cx3[i][j][10] = _mm_setzero_si128();
			x3[i][j][11] = 0.2104627937078476f;
			cx3[i][j][11] = _mm_setzero_si128();
			x3[i][j][12] = -0.059749942272901535f;
			cx3[i][j][12] = _mm_setzero_si128();
			x3[i][j][13] = -0.5793862342834473f;
			cx3[i][j][13] = _mm_setzero_si128();
			x3[i][j][14] = 0.5359512567520142f;
			cx3[i][j][14] = _mm_setzero_si128();
			x3[i][j][15] = -0.21632997691631317f;
			cx3[i][j][15] = _mm_setzero_si128();
			x3[i][j][16] = -0.0271208006888628f;
			cx3[i][j][16] = _mm_setzero_si128();
			x3[i][j][17] = 0.009552535600960255f;
			cx3[i][j][17] = _mm_setzero_si128();
			x3[i][j][18] = -0.05762327089905739f;
			cx3[i][j][18] = _mm_setzero_si128();
			x3[i][j][19] = 0.0022283289581537247f;
			cx3[i][j][19] = _mm_setzero_si128();
			x3[i][j][20] = 0.35228079557418823f;
			cx3[i][j][20] = _mm_setzero_si128();
			x3[i][j][21] = 0.22974683344364166f;
			cx3[i][j][21] = _mm_setzero_si128();
			x3[i][j][22] = 0.1878991425037384f;
			cx3[i][j][22] = _mm_setzero_si128();
			x3[i][j][23] = 0.14676685631275177f;
			cx3[i][j][23] = _mm_setzero_si128();
			x3[i][j][24] = 0.017987212166190147f;
			cx3[i][j][24] = _mm_setzero_si128();
			x3[i][j][25] = 0.028198005631566048f;
			cx3[i][j][25] = _mm_setzero_si128();
			x3[i][j][26] = -0.17409856617450714f;
			cx3[i][j][26] = _mm_setzero_si128();
			x3[i][j][27] = -0.23908104002475739f;
			cx3[i][j][27] = _mm_setzero_si128();
			x3[i][j][28] = 0.03782324865460396f;
			cx3[i][j][28] = _mm_setzero_si128();
			x3[i][j][29] = 0.14200152456760406f;
			cx3[i][j][29] = _mm_setzero_si128();
			x3[i][j][30] = 0.03421904146671295f;
			cx3[i][j][30] = _mm_setzero_si128();
			x3[i][j][31] = 0.4172836244106293f;
			cx3[i][j][31] = _mm_setzero_si128();
		}
	}
	for (int ix = -1; ix < 7; ix += 1)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 1) / 1;
		for (int jx = -1; jx < 7; jx += 1)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 1) / 1;
			x_1 = ix + 0;
			if (x_1 >= 0 && x_1 < 8)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 12, -16, 23, -5, 6, 0, -11, 0, -9, 5, -18, -5, -3, -3, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 7, 17, 16, -49, 0, 16, -1, -4, 7, 6, -18, -16, -1, -5, -20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -5, -10, -33, 5, -9, -1, 17, -7, -1, -38, -31, 17, 1, 7, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(10, 10, -12, 18, -5, 32, -3, 0, -1, 6, -18, 17, -21, 3, -4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, -5, 3, 19, -1, 6, 22, 36, 17, 22, 11, 0, -10, -7, -25, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, 5, 2, 20, 9, 4, -3, 6, 2, 9, -4, -3, 9, -5, -1, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(14, 5, 10, -35, -4, 8, 9, 13, -3, 2, -8, -1, 9, 3, 0, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, 11, -28, 14, 10, -15, -17, 3, 6, 10, -12, -9, 15, -2, -18, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 2, 16, -1, 11, -51, 27, -25, -2, 8, 8, 8, -18, -21, 0, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(15, -13, -7, -6, 0, -7, 15, 5, -17, -11, 5, -10, -10, 4, -2, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(10, -35, -14, -30, -5, -47, -12, -15, 10, 0, 18, 1, -5, -3, 4, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -6, -9, -1, 13, 4, 5, 4, -4, -6, 6, 0, 6, -6, -1, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -3, 0, 3, -3, -7, -19, 8, -14, -10, 2, -6, -36, 20, 11, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, -7, 11, -3, 1, -14, -3, 11, -20, -6, -1, 0, -4, 1, 0, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, 16, -35, -4, -20, -8, 7, 15, 7, 6, -23, -13, -127, -46, -80, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 1, -6, 0, 1, -3, -23, 2, -9, -7, 3, 4, -14, 2, 2, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 15, -28, 3, -7, 17, -13, -4, -8, 11, 7, 24, -27, -14, -51, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(11, -30, 2, 0, -5, -1, -6, -12, -17, 7, 8, 0, 6, 0, -8, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-20, -9, 5, -24, -17, 27, 17, 36, 4, 2, -26, -14, -15, 1, -1, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(10, 6, -24, -26, 24, 0, -6, 3, -10, -26, -12, -12, 27, 1, 9, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 0, -16, 24, 13, -3, 8, -1, -1, -10, 10, 6, 0, -49, -29, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, -4, 23, 3, 8, -15, 16, -12, 18, -7, -2, -6, -24, 3, -3, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, -17, -24, -21, 6, -9, -39, -9, -19, -8, 3, -5, -20, 6, 13, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -17, -74, 1, 4, 8, -18, 15, -8, -25, 10, 1, 3, 5, -8, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -3, -8, 8, -2, -3, 7, 15, -14, -10, 0, -3, 14, 2, 4, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -12, 2, 3, 4, -5, 2, 6, -17, -11, 21, -4, -1, 10, 5, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -12, 0, 6, 10, -22, 36, -10, 0, 7, 24, 4, -7, 4, 3, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -25, -10, 0, -1, -3, 9, 29, 4, -1, -29, -15, 8, 6, 5, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(19, -38, -9, 21, -2, -27, -7, 12, -5, -15, 21, -33, -17, -3, 10, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, -13, 23, 12, -12, 37, 10, -3, 7, -32, 11, 2, 6, -15, -12, -41);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-36, -30, -12, -27, -13, -3, 4, 5, 9, -17, -19, -14, 4, -6, -6, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, 1, 22, -4, 3, -2, 13, 18, 5, 0, 5, -2, -12, 4, -5, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, 11, 3, 4, -7, -1, -25, -3, -1, 0, -9, 10, 7, 9, -17, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, 5, 14, 5, 1, 7, -2, -15, -10, 3, 8, 3, -22, 2, 1, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -16, 37, -1, -1, -1, 17, 21, 6, -3, 7, -7, 1, -7, 4, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 16, -19, -23, -6, 4, -13, 9, 1, 11, 8, -5, 33, 7, 8, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-9, -18, 0, -21, 14, -9, -22, 7, -10, 1, -9, 0, -2, -21, -10, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, -3, 11, -25, -7, -7, -1, -4, 5, 1, -2, -21, 10, 7, 12, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -17, 4, -5, -12, -6, 12, 16, 0, -12, 10, -9, 2, -7, 4, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -5, -1, -9, -3, 16, -7, 7, 2, 0, 16, 12, 3, 4, 0, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, 9, 15, -4, 5, -48, 15, -26, -3, 7, 1, -17, -24, -13, -13, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -14, 23, 33, 4, 16, 9, 12, -20, -1, -5, 0, -26, -16, 5, -24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(12, -19, -6, -5, -1, -22, 15, 0, 9, -32, -15, -27, -18, 14, 5, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, -1, 0, 16, 13, -5, 18, 8, -11, -10, 5, -6, -9, -11, -4, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -22, 4, 5, 2, -40, 16, -7, -4, -14, 11, -5, -21, -2, 0, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, -9, 17, -14, -2, -8, 16, 0, -9, -6, -1, 0, 8, -7, -4, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(11, 10, 9, -27, -35, -4, 5, 7, 15, -9, -46, -60, -56, 1, -28, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 6, 1, -19, 4, 1, -10, 4, 1, -4, -1, -3, -10, -7, 2, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, 21, -85, 14, 12, -32, 6, -16, -9, 1, -11, -17, -18, -16, -40, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, -3, -3, -4, 4, -7, 2, -20, -31, -7, -24, -2, -10, 9, 5, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, -3, 4, -38, 1, -1, -24, -3, -10, -1, -15, 2, -8, -5, -1, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -3, -24, 23, 3, -36, 11, -8, -9, -11, 14, -2, -5, -5, 0, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-20, 19, -20, 33, 17, 8, -14, 0, -1, 4, -10, -1, -12, -13, -27, 23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -4, 10, 24, -11, 25, -17, 10, 18, -15, 5, -5, 4, -9, -4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, 8, -20, 5, 6, 11, -20, -8, -22, 1, -28, 10, -15, 0, -13, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -19, -17, 2, 3, 10, -3, 6, -7, -3, 14, -5, -9, 4, 9, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -15, 0, -6, -12, -7, -10, 14, -6, -18, -6, -5, -18, -3, 5, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -23, 7, -20, 3, 10, -20, -2, -17, -14, -5, -11, -16, -2, 7, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 0, -4, 26, 11, -10, 18, -3, -2, 0, -12, 5, -17, -5, -5, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -57, 0, -7, -7, 0, -5, 21, -8, 0, -39, -1, 9, 12, 10, -34);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -33, -11, -14, 10, -22, 41, -3, 0, -11, 14, -35, -3, 3, 17, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -4, 36, 6, -24, 11, 1, 4, -9, -23, 1, -18, -13, -1, -12, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, -5, 23, 36, 16, 25, 11, 18, 1, 0, -21, 14, -2, 8, 20, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, -34, -25, -5, 0, 4, -17, 25, -4, -8, 27, -20, -19, -21, -18, -31);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, 4, 10, -32, -10, 19, 24, 1, 13, 13, -5, 0, 7, 17, -9, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(13, 15, 3, -3, 7, 8, 31, 16, 9, -6, 0, -9, -9, -2, 0, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, 9, 45, 20, 0, 6, 0, 1, 4, 9, -8, 2, -10, -33, -15, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-28, 3, -10, -2, -9, 0, -1, 3, 1, 1, 19, -1, 7, -3, -4, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -11, -17, -15, 0, -10, 16, -16, 8, 4, 11, -2, 18, 8, 16, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, -5, 35, 20, -11, 13, 18, 10, 12, 0, 19, -2, 2, -20, 4, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-41, 12, 14, 14, -1, 5, 0, 10, 5, 12, 1, 5, -13, -27, -12, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, 1, -11, 2, -4, -1, 0, 2, -15, -7, 17, 4, 11, 19, 13, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -5, -3, -27, -27, -37, -18, -25, 2, 8, -24, 7, -1, -8, -1, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -16, -4, 20, 6, 3, -12, 18, -21, -22, 8, 0, -15, -22, -18, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(25, -38, -1, -1, -7, 7, -8, 12, -9, -17, 11, -13, -1, 17, 14, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, 3, -1, 0, 6, 8, 10, 8, -18, -5, 0, 3, -13, -1, -5, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -7, -23, 39, -3, -67, -11, -74, -7, 0, 48, -13, 3, 3, -2, 24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, 0, 18, -5, -5, -1, 15, 0, 7, -6, 0, -3, -7, 2, 0, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 6, -43, 1, -41, 7, 3, 1, 28, 0, -15, -23, 5, 19, -13, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, -9, 19, -41, 0, -8, 0, 21, -7, 2, -20, 6, -5, 0, 9, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, 0, -32, -4, -17, -27, 2, -13, 18, 6, -36, -10, -7, -9, -5, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 8, 0, 5, 3, -5, -45, -19, -25, -20, -7, 1, -4, 14, 4, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -3, -2, -44, -18, -15, 14, -12, 6, 9, -5, -1, 0, 4, 18, -35);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -1, -22, 6, 18, 14, -11, 6, -10, 10, 14, 1, -11, 0, 7, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(20, 12, 13, -8, 21, 7, 10, 11, -5, 4, 0, 3, -12, 9, 3, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -13, -9, -6, -3, 18, 0, 25, 23, -13, 4, -12, 15, -47, -15, -32);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -1, -55, -17, 4, 1, -15, 6, -23, -15, -14, -10, -6, 11, 5, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -6, -28, 6, 12, 22, -25, 7, -26, 2, -6, 1, -24, -12, -4, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 20, 2, 4, 5, 6, 17, 16, 7, 1, -30, 3, -33, 9, 11, -37);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -23, 18, 4, 23, 7, 3, 7, -17, 0, -7, 1, 5, -4, 4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -20, -8, 10, 16, -5, 5, 0, -6, -1, -3, 0, -31, -12, 3, -23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -41, 17, -20, -7, 8, 14, 12, -4, 0, -1, 1, -8, -5, 2, -24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(20, -34, -5, 16, 12, -19, 17, 11, 20, 7, 24, -10, 12, 30, 29, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, -9, 29, 0, -25, -29, -29, -15, 0, 23, -10, -1, 13, -8, -27, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 0, 2, 0, -6, 10, 5, 7, -9, -2, 15, -1, -24, -11, -8, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -29, 16, -15, 8, 6, -39, 17, 0, -9, -20, -35, -64, 9, 12, -43);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
			}
			x_1 = ix + 1;
			if (x_1 >= 0 && x_1 < 8)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, -8, -26, -4, 2, 1, -13, -13, -18, -6, -5, 7, 33, 4, 12, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, -2, -4, 9, -3, 0, 11, -12, 1, 4, -15, -13, -18, -5, -5, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -15, 15, -35, 0, 6, 0, 2, 12, 11, 10, -25, 0, 35, -22, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 14, -13, 14, -1, 34, -7, 17, -16, 5, -30, 21, 9, 2, -3, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 18, 10, 16, -3, 17, 9, 7, -7, -8, 3, -11, 18, 5, -4, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, -4, 29, 12, 22, -6, -24, 13, 0, 0, -10, -43, 19, 0, -8, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -19, -13, -27, 4, 0, 2, 5, 6, 0, 15, -9, 10, 12, 5, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 2, 18, 6, -8, -5, 8, 6, -3, -3, 10, -21, -16, -9, -16, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 11, 8, 10, -8, -21, -9, -37, 0, 19, -16, -3, -26, 11, 3, 16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -6, -14, -11, 5, -9, 8, -3, -8, 0, -3, -10, -16, 11, 6, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, 0, -24, 16, 7, -8, 39, 0, 19, 10, 35, 0, -3, -7, -6, 11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -8, -23, 1, 7, -7, 8, 0, -2, -4, 16, -6, 3, 10, 2, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 0, 10, -25, 4, -1, 18, -1, 5, -13, -8, -2, -21, 17, 7, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -19, 29, -12, 6, -25, -18, -1, -16, -6, 8, -24, -18, -2, -3, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-51, -9, -46, 0, 12, -14, -40, 1, -3, -5, -40, -3, -73, -72, -18, -59);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -17, 13, -37, -4, 7, 15, 0, 14, 2, 1, 12, 1, 0, 1, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, 0, 1, -2, -6, 26, -53, -7, -23, 8, -36, 4, -38, -8, 4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, -5, 1, -34, -2, -11, -3, 5, 2, 2, -10, -6, 8, -2, 5, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, 5, 13, 15, -2, -10, 15, -11, -1, 2, 5, 5, 1, -15, -11, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 4, -56, -24, -11, 6, -70, 3, -31, -22, -22, -7, 0, 19, 12, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -59, -21, -23, 7, 0, 31, -14, -2, 4, 20, -16, 0, -12, 3, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -18, 10, 18, -7, -20, 13, -40, 19, 16, 33, 5, -23, -27, -4, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, 14, 0, -1, 13, 7, 0, -14, 1, 12, 12, 15, -15, 10, 8, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, -2, -58, -19, 12, 5, -43, 2, -23, -10, 15, 7, -9, 3, -4, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 4, -8, 27, 0, 11, -5, 13, -1, 17, -25, -13, 22, 0, 10, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, 0, 10, 27, 28, -30, 24, -31, 8, 16, -15, 14, -9, -4, -4, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 10, 5, 19, 16, 14, 23, -8, 6, 33, 2, 17, 18, 2, -5, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 2, -12, 1, -10, 12, 4, -1, 8, -11, -8, -19, -12, 0, 16, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 4, -38, -12, -8, -11, 12, -7, 6, 7, -11, 5, 1, 6, 5, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 4, -27, 5, -1, 18, 11, 5, -17, 15, -16, 8, 5, -1, 10, -23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, -17, 12, -39, -3, 0, 12, -3, -2, -8, 3, -16, -7, 17, -4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, -43, 2, 7, -3, -12, -39, 8, 24, 6, 0, 6, -19, -12, 9, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -6, -23, -8, 4, -15, -21, -3, 3, -21, 28, -20, 18, 5, 3, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, -1, -11, 9, 5, -22, 6, -34, -8, 8, 0, -7, -3, 1, 4, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 2, 9, 11, -22, 24, 5, 5, 10, 8, 0, 9, -13, 20, -9, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-43, -18, -17, -25, -6, -8, 5, 5, 4, 5, -24, -17, -1, 4, -2, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, 3, 23, -13, -10, -17, -27, 4, -6, -1, -2, -7, 13, -3, -34, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -12, -27, -44, -8, 0, -19, -3, 5, -9, 17, -20, -11, 12, -41, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-9, -15, 54, 1, -16, 9, 0, -14, 7, -10, 2, -6, -1, 13, -15, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-9, -8, 30, -32, -9, -18, -12, 9, -3, 11, -18, -12, -5, -17, -14, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(20, -2, 5, 31, 27, -40, -5, -34, -4, 16, 0, 18, -31, -26, -5, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -13, 5, 32, 9, 2, 36, 0, -4, 17, -26, 9, -3, 2, 14, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -27, -19, -6, -7, -19, 0, 0, 4, -2, 4, -17, 1, 2, 12, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -10, -1, 1, -3, -12, -4, -12, -12, -2, 0, -6, -10, 7, 2, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-20, -1, 0, -6, -3, -5, 28, -10, -5, 6, 6, 0, -15, -7, -4, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -10, 8, -37, -13, -9, -1, -3, 2, -6, 3, -8, 2, 8, 10, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-34, 0, -64, 0, 0, -9, 1, -8, 10, -3, -12, -37, -56, -21, -21, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, 3, -5, -10, -10, -6, -22, -8, 2, 0, -21, 1, -10, -14, -18, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, 4, 0, 4, 6, -7, -6, -24, -8, 1, -31, 9, -38, -32, -17, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -8, 7, 11, 15, -1, 14, 6, -3, -6, 23, 0, -2, -3, 1, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, -9, 47, 7, 5, 9, -28, 8, -2, 5, -18, 3, 6, -10, -28, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-31, -15, -13, -27, 0, -51, -22, -62, -30, -1, -19, 0, -8, -1, -1, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-45, -4, -6, -10, 17, -1, -35, -7, -6, 0, -9, 0, -10, -47, -38, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-39, -18, -20, 20, 1, -7, 12, -11, 10, -2, 8, -21, -13, -46, -11, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, 12, -24, 12, 0, 8, -20, 1, 6, 10, 10, 7, 4, -1, -20, 35);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, 4, -59, -11, 4, -12, 0, -16, -12, 5, -36, -5, -15, 8, 7, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, -3, -21, -2, -6, -5, -9, 19, 5, -8, 0, -20, -17, 17, 11, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, 7, -7, 5, -7, -9, 16, -4, -8, -2, -10, -13, -13, 0, -7, 11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 14, -20, 11, 0, 25, 2, -1, 8, 16, -7, 13, -7, -18, -28, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 4, -10, 8, -17, 5, -2, -2, 11, -1, 3, 0, 9, 20, 4, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(16, -14, -35, 16, -2, 3, -3, 6, 2, -5, 15, -17, -19, -7, 19, -29);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -4, -6, 1, 6, -3, 0, 0, -11, -15, 22, 0, 30, 7, 6, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -8, 5, 12, 7, -1, 23, -5, -5, 16, -21, 5, -13, -5, -1, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, -26, 4, -5, -17, -1, -79, 4, -12, -3, 8, -11, 9, -17, 4, -74);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -26, 55, -23, -10, -7, -1, 13, 4, 7, -1, -8, -8, -11, -17, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, 1, -8, 21, 1, -7, 1, -20, -1, -13, 8, 0, -7, 4, -5, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 25, -14, 6, -20, -15, -8, -5, -24, -4, 0, -6, -10, 5, -15, 20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, 0, 7, 13, 0, -1, 1, 2, 4, 4, 12, 8, 0, -35, 2, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -29, 18, -27, 7, -12, 16, -15, 0, 5, 9, -2, 0, -32, 3, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, 12, -8, 14, -3, 9, -7, 0, -15, -3, 13, 11, -9, -13, 4, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 21, 35, 15, -6, 0, 0, -11, -2, 2, 9, -20, -10, 1, -19, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -27, 9, 6, 13, -13, 17, -7, -2, 5, -10, -7, 0, -15, 4, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 14, 2, 18, -2, -11, -11, -32, 10, 10, -4, -7, -28, -7, -3, 11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-31, -54, -11, 1, 14, -13, -9, -1, -3, 1, 22, 1, 4, -30, -11, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, -38, -8, 5, 6, -4, 6, -4, -22, -14, 19, -25, -7, 12, 13, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -18, 3, 40, 21, -4, 1, -9, -10, 13, -7, 9, -19, -10, -3, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 13, -44, -19, -11, -20, -11, -16, -4, -3, 27, -18, -8, -26, -37, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -24, -1, 7, -8, 3, 7, -6, 3, -14, 5, -2, 6, -6, -9, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 10, -72, 8, 5, 8, 6, -7, 22, 15, 9, -5, 10, -11, -11, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, 3, 1, -4, 0, -11, -18, -6, 1, -11, -13, -2, -12, -15, -2, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(18, 7, -9, 11, 17, -16, -8, 0, 0, 16, 0, 1, -9, 7, 4, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, -6, -3, 15, 7, 13, 14, -5, 6, 17, -2, 3, -18, 7, 1, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, -13, 1, -61, 0, 0, 6, 6, 17, 9, -4, -7, 17, -19, 2, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -4, -40, -5, 13, -51, -51, -36, -26, -10, -8, -9, 0, -3, -1, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(8, 18, 1, -10, 10, 0, -15, 11, -9, -5, -8, 6, -10, 0, -10, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -97, -33, -9, -1, -10, -21, 7, -13, -5, 6, -20, 3, 10, -5, -29);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, 15, -33, -5, 11, -3, -14, 3, -7, -8, 13, -5, 0, 13, 0, 25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 15, -27, 26, 19, -5, 0, -12, -24, 7, 14, 10, -5, -13, -15, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 13, -20, 5, -7, 19, 25, -1, 15, -2, 0, 12, 6, -4, -10, 11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(17, -14, 9, -9, 2, -10, 6, -6, 5, 3, 9, -4, 7, 0, 15, -16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 21, -16, 3, -3, 9, -2, 2, 16, 24, -1, 7, -33, -2, -1, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, -13, -2, 9, -3, -4, -13, 4, -8, -19, -8, -2, 0, 3, -9, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-28, -9, -54, 10, -5, -9, 10, -7, 4, -9, -4, -17, -1, -13, -5, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, -9, -5, -21, 5, -23, 0, -16, 5, -11, -6, -34, 16, 0, -6, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(12, 13, 20, -2, -17, -8, -13, -13, -8, -16, 0, -9, -25, -6, -8, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, -2, 4, 25, 6, -4, -76, -3, -15, -7, 47, -14, -90, -1, 2, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
			}
			x_1 = ix + 2;
			if (x_1 >= 0 && x_1 < 8)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, -9, 14, 24, 10, 24, -1, 1, -29, 14, -42, -5, -14, -25, 2, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-17, 7, 20, 11, -14, 25, 5, 3, 9, -1, 7, 9, -4, -1, 4, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 22, 21, -24, -17, -2, -20, -2, 7, -2, -29, 3, -22, -30, -31, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, -9, 29, -18, 15, -5, -15, 11, -28, -9, -28, -45, -28, -15, -31, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(12, 8, 0, 5, 1, 4, 13, 8, -21, -11, -4, 1, 13, -1, 16, -28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 5, 26, 12, 9, 4, 1, 1, -7, -27, -4, -22, -5, -7, -11, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, 27, 14, -26, 0, 15, -33, 4, 3, 10, 8, 11, 0, -3, -37, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(22, 0, 0, 3, 13, 4, 23, 8, -10, -37, 14, -11, -14, 0, 4, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-17, 6, -22, 0, -11, -30, 9, 1, 7, -10, -14, -53, -45, -4, -19, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-30, 15, -32, 2, 6, 12, -3, 0, -5, 0, -19, 13, -29, -1, -13, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(19, -53, -12, 11, 24, -15, -6, -10, 7, 14, 36, 11, 11, -17, -8, 16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 8, 1, 8, 24, 3, 7, -5, -4, 0, 10, 0, -15, -1, 2, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(24, 0, 15, 10, 8, 0, 13, 14, 11, 0, -34, 5, -18, 8, 6, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, -3, 3, -4, 8, 15, 5, 10, -9, -8, 6, -4, 6, 6, 3, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 4, 4, 14, 17, 10, 23, 47, -11, 18, 17, -26, -47, -35, -16, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, 1, 5, 8, -2, 23, 17, -4, 8, 2, 6, 3, 24, -1, 6, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 14, 12, 3, -17, -5, 7, -5, 0, -13, -8, -2, -25, 4, -6, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 10, 13, 5, 11, 26, 2, 7, -3, -3, 1, 0, 5, 7, -17, 23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(12, 12, 4, 11, -13, 5, -14, -9, -10, -16, 16, -6, 8, -4, 19, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 28, -10, 6, 12, 19, -13, 9, -9, 0, -20, 3, -63, 3, 0, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-34, -7, -68, -7, -10, -21, -29, -31, 8, -10, -6, -9, 8, 18, 15, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, -13, -21, -18, 29, -31, 1, -57, 10, 42, -3, 13, -17, 2, -17, 37);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 2, -30, 22, -5, -17, 32, -38, 0, 18, 8, -9, -26, 4, 5, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 20, -21, -29, 5, 8, -27, 0, -29, -8, 19, 15, 8, -10, -10, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -5, -11, 21, -21, -26, -17, -21, -5, -9, -2, -22, 25, 10, 2, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-34, 1, -19, 20, 10, -34, 20, -26, 9, 34, -9, 22, 6, -25, -20, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, -27, -31, 25, -40, 4, 4, -6, -1, -34, 9, -13, 0, 0, -2, -20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 3, -22, -2, -14, -8, -1, -37, 9, -4, 9, -1, 19, 10, 9, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, -30, 1, 17, 5, 1, 21, 0, 18, -7, -2, -5, 4, 8, 10, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, -15, 0, 30, 8, 11, 10, 8, -17, 7, -23, -2, 3, -10, 6, -49);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-27, -6, -6, -4, 5, -9, 7, 0, 15, 1, 3, -2, 1, -5, 0, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -8, 1, -16, -14, -10, 15, -9, 16, 10, 19, 9, -7, 16, 15, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, -22, -7, -19, 15, 0, 4, 1, -12, -10, -6, -41, -16, -14, -23, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 8, 15, 11, -9, -18, 7, -20, 15, 8, 10, -31, 16, 9, 9, 18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 9, -3, 12, -5, 1, 0, -4, -13, 16, -27, 12, -33, -27, 8, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-34, -26, -8, -23, -22, -2, 9, 0, -6, -11, -5, -5, 0, -22, -89, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -21, -15, -17, 10, -11, -10, 4, -23, -32, 15, -15, 8, -8, -4, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, 10, 21, -23, -1, -6, -18, -4, -1, 10, -18, -3, -16, -29, -28, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, -10, 7, 7, -4, 2, -6, -3, -1, 1, -26, 13, -8, -2, 2, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -6, 22, -30, 0, 8, 0, 14, -14, -32, 9, -10, 9, -5, -9, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, 3, -27, 16, -34, -36, -18, -23, 6, 2, -5, -12, -27, 0, -21, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, 15, -35, 10, 5, 1, 22, 4, 8, 15, 1, 12, 2, -6, 3, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, -34, -1, 7, 19, 18, 2, 9, -4, 7, -10, -22, -33, -6, 2, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, 12, -41, -1, -1, -5, 25, -6, -1, -5, -16, -22, -31, 0, 7, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -17, 5, 14, 2, 6, 0, -5, 7, 14, -2, -13, -14, 11, 16, -24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, 14, 0, -8, -5, 14, -27, 0, 5, 3, 10, -10, 3, 10, 0, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-33, -22, -27, -23, -3, -15, -12, 14, -12, -25, -8, -84, -88, -15, -17, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 8, -5, 14, 6, -15, 0, -8, -10, 13, 11, -8, 8, 10, 0, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, 7, -11, 24, -8, -17, 1, -19, -2, 12, 16, 11, 1, 6, -12, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(16, -16, 23, -21, 1, 29, -24, 1, -27, 0, 24, -15, -23, 0, -7, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, -7, 0, -8, -15, 7, -24, 4, 2, -7, 0, 11, 11, 1, -14, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, 23, -31, 5, -18, -25, 7, -13, -9, 13, -1, 4, -12, 1, 4, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-25, -40, -38, -21, -14, -6, -22, -27, -1, -4, -16, 1, 18, -1, -15, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, -3, -17, -9, 10, 1, -20, 14, -7, 0, 0, 14, -10, -17, -10, -39);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, -6, -24, 5, 6, -3, 25, -5, 3, 9, 16, 12, 15, -23, -20, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 28, -53, 4, -6, -12, 29, -11, -14, 8, -12, -17, -10, 9, 2, 25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-22, -12, -34, 18, -14, -14, -2, -11, 0, -15, 8, 8, -16, -6, -23, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -6, -9, 18, 17, -17, 5, -4, 0, 0, 0, 6, -17, -17, 0, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, -11, 6, 5, -6, -6, -14, -14, -20, -8, -23, 4, -13, -39, -22, -37);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, 7, -10, 15, -8, 3, -3, -34, 1, 0, -5, 10, 12, 0, 0, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -19, 0, 17, -1, 9, -16, 14, -4, -28, 13, -17, -28, -7, 23, -50);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, -13, -13, 4, 10, 1, 8, 3, -33, -3, 2, -11, 3, -7, 6, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, 9, 6, 14, 4, 6, 12, 12, 6, 11, -20, 15, -20, -4, 9, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-32, -2, -5, -5, -15, -14, -26, -15, 4, -5, -9, 2, 9, 1, -11, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 5, 60, 2, 3, 15, 15, 9, 0, 0, -2, 12, -6, -42, 1, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -16, -17, 5, 13, -6, 1, -6, -12, 3, 14, 5, 29, 6, -16, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 0, -5, -1, 0, -8, 1, -7, 7, -9, -25, 5, 5, -6, 11, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 16, 25, 25, -11, 4, 18, -2, -20, -2, 3, 9, -28, -17, -2, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -27, 30, -4, -2, -4, 23, -1, -7, -1, -14, -8, 2, -32, 7, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 0, 5, -6, 6, 5, 0, 2, -14, 15, -20, 15, -19, -21, 19, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(7, 5, -3, 0, -3, -4, -3, -6, 0, -14, -15, -10, 3, 2, 7, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 6, 27, -13, -4, 2, 10, 8, 0, -39, -5, -14, 12, -3, -5, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 21, -36, 0, 5, -27, 9, -45, 14, 6, -16, 1, -17, 4, 0, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 0, -52, 1, -8, -19, 8, -2, -5, -8, 17, -11, -16, 0, 0, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -34, 9, 0, 22, 17, 6, -5, -21, 7, 5, -4, -33, -16, 4, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, 4, -31, 10, -6, -10, 22, -8, 1, 18, 0, 16, 11, -16, -9, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(26, -34, -2, 6, -9, -28, -16, -35, 4, 0, 14, -8, -11, -6, -7, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(11, -10, 14, -24, 4, 27, 5, 7, 21, 5, 6, 7, 4, 14, 7, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(6, -13, -61, -27, 20, 2, 33, 10, 1, 9, 5, -26, -7, 0, 0, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, -3, -13, -21, 24, -5, -50, -3, -10, 7, 17, 11, 11, -2, -9, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, 21, -2, -7, -5, -8, 4, -16, 19, 0, 9, 7, -5, -1, 3, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -14, 21, 0, 0, 1, 23, 0, 6, 1, -19, 7, 28, -4, -14, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-22, -34, 35, -21, 8, 8, 16, -7, -4, 11, 0, 0, -14, -48, -21, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, 5, -42, 0, -9, -66, -8, -63, -9, 20, 0, 10, 6, 29, 13, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 18, -9, -3, 14, -12, 15, -9, -24, -18, 28, 8, 18, -4, -17, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -16, -13, -16, 9, -3, -38, 14, -6, -19, -16, -33, -43, 4, 3, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -3, -1, -27, 2, -4, -22, 9, -11, -16, 21, 3, 10, 7, 3, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(4, -5, -32, 5, 3, -4, 30, -14, -13, 18, 24, 7, 21, 2, 20, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 19, 4, 9, -39, 9, -9, -11, -27, 4, -17, 27, -25, -19, 8, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, -4, 4, 0, 13, -10, 21, -3, 25, 14, -14, 5, -12, -5, 8, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-22, 14, -8, 14, -3, 2, -20, -2, 9, -12, -11, 16, -57, -18, -24, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 3, 12, 4, -14, 2, -34, -19, -4, 6, -14, 0, -6, 12, 18, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, -26, 4, 21, -15, 3, -5, 0, -19, -22, -10, 11, -6, -34, -7, -28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -7, 0, -29, 8, 9, 7, -2, -7, -39, -7, -18, -7, -5, 7, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, 16, -5, -6, 1, -14, -21, -26, -10, 6, -6, 0, -27, 1, 0, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in3[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -10, -9, -60, 7, -23, 2, -11, -2, 6, -34, -3, -39, -24, 3, -24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx3[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx3[x_out_1][x_out_2][31]);
				}
			}
		}
	}

	for (int i = 0; i < 8; i++)
	    for (int j = 0; j < 8; j++)
	        for (int k = 0; k < 32; k++){
	            qx = cx3[i][j][k];              
	            lo = _mm_srai_epi32(_mm_unpacklo_epi16(qx, qx), 16);
	            hi = _mm_srai_epi32(_mm_unpackhi_epi16(qx, qx), 16);
	            sum1 = _mm_hadd_epi32(hi, lo);
	            sum2 = _mm_hadd_epi32(sum1, sum1);
		        _mm_store_si128((__m128i*)res, sum2);
	            x3[i][j][k] += (res[0] + res[1]) * 0.02742383986946166f * 0.008748259395360947f;
	        }
	for (int i = 0; i < 8; i += 1)
	{
		for (int j = 0; j < 8; j += 1)
		{

			x = _mm_load_ps((float*)&x3[i][j][0]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][0], x);

			x = _mm_load_ps((float*)&x3[i][j][4]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][4], x);

			x = _mm_load_ps((float*)&x3[i][j][8]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][8], x);

			x = _mm_load_ps((float*)&x3[i][j][12]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][12], x);

			x = _mm_load_ps((float*)&x3[i][j][16]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][16], x);

			x = _mm_load_ps((float*)&x3[i][j][20]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][20], x);

			x = _mm_load_ps((float*)&x3[i][j][24]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][24], x);

			x = _mm_load_ps((float*)&x3[i][j][28]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x3[i][j][28], x);
		}
	}
	static float x4[4][4][32] = {0};
	for (int ix = 0; ix < 7; ix += 2)
	{
		int x_1, x_out_1;
		x_out_1 = ix / 2;
	for (int jx = 0; jx < 7; jx += 2)
	{
		int x_2, x_out_2;
		x_out_2 = jx / 2;
		x = _mm_load_ps((float*)&x3[ix][jx][0]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][0], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][0], x);
		x = _mm_load_ps((float*)&x3[ix][jx][4]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][4], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][4], x);
		x = _mm_load_ps((float*)&x3[ix][jx][8]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][8], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][8], x);
		x = _mm_load_ps((float*)&x3[ix][jx][12]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][12], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][12], x);
		x = _mm_load_ps((float*)&x3[ix][jx][16]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][16]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][16]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][16], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][16]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][16]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][16], x);
		x = _mm_load_ps((float*)&x3[ix][jx][20]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][20]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][20]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][20], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][20]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][20]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][20], x);
		x = _mm_load_ps((float*)&x3[ix][jx][24]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][24]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][24]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][24], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][24]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][24]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][24], x);
		x = _mm_load_ps((float*)&x3[ix][jx][28]);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 0][28]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 0][jx + 1][28]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][28], x);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 0][28]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x3[ix + 1][jx + 1][28]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x4[x_out_1][x_out_2][28], x);
		}
	}
	static float x5 alignas(16) [1][1][2] = {0};
	static __m128i cx5 alignas(16) [1][1][2];
	static unsigned char cx_in5 alignas(16) [4][4][32];

	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 4; j++)
	        for (int k = 0; k < 32; k++)
	            cx_in5[i][j][k] = x4[i][j][k] / 0.020375443622469902f;

	for (int i = 0; i < 1; i += 1)
	{
		for (int j = 0; j < 1; j += 1)
		{
			x5[i][j][0] = 1.7176446914672852f;
			cx5[i][j][0] = _mm_setzero_si128();
			x5[i][j][1] = -1.7176449298858643f;
			cx5[i][j][1] = _mm_setzero_si128();
		}
	}
	for (int ix = -0; ix < 1; ix += 1)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 0) / 1;
		for (int jx = -0; jx < 1; jx += 1)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 0) / 1;
			x_1 = ix + 0;
			x_2 = jx + 0;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-30, -28, -25, -7, 5, 1, 4, 4, -58, -42, -70, -37, -53, 7, -6, -29);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(25, 27, 26, 12, -1, -2, -5, -2, 55, 38, 73, 32, 54, -3, 5, 30);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(11, 4, 7, -15, -19, 12, 1, -27, 0, 4, 12, 3, 0, -82, 12, 3);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-11, 0, -8, 15, 22, -7, 0, 24, 0, -4, -12, -3, 3, 82, -13, -5);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-18, 3, -22, 3, 0, 9, 3, -1, -76, -50, -81, -81, -25, -76, 0, -58);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(19, -3, 16, 0, -2, -9, -1, -3, 74, 49, 75, 75, 26, 75, 0, 56);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-7, -12, 12, -8, -12, -2, 1, -23, 5, 0, -14, 17, -1, -73, 9, 6);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(12, 11, -14, 9, 9, 6, 3, 21, -11, -5, 12, -18, 3, 71, -11, -10);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 2;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-21, 5, -15, 12, 0, 17, 1, 0, -20, -53, -63, -46, -30, -54, -24, -75);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(18, -2, 15, -11, 0, -15, 0, 3, 16, 50, 62, 42, 31, 58, 28, 78);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-18, 0, 16, 4, -23, -5, 0, -26, 2, -1, -24, 14, 3, -19, 3, 3);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(18, 1, -11, -3, 21, 8, -2, 27, -2, -2, 20, -8, -6, 19, -7, -9);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 3;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-23, 13, -43, 10, 6, 3, 2, -2, -18, -53, -76, -51, -50, -42, -45, -97);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(17, -12, 44, -9, -2, 0, -7, 5, 23, 55, 80, 51, 50, 41, 49, 102);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-21, 6, 1, 22, -4, -4, 2, 6, 1, 2, -17, 9, 0, -31, 15, -14);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(24, -4, -3, -22, 1, 3, 0, -5, -3, -1, 15, -4, 0, 29, -16, 14);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_1 = ix + 1;
			x_2 = jx + 0;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-1, 11, -36, 5, 7, -5, 11, -7, -34, -57, -37, -97, -37, -7, -1, -63);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(3, -10, 36, -3, -7, 7, -16, 5, 40, 64, 35, 94, 33, 9, 2, 64);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(4, 11, -40, 0, -11, 14, 9, -17, 0, 4, 17, -5, -5, -56, 2, 2);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-6, -16, 42, 4, 12, -21, -4, 19, -2, -7, -16, 2, 5, 51, -4, -8);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-15, 29, -53, 15, -1, 11, 11, 10, -46, -61, -17, -37, -31, -43, -16, -60);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(14, -29, 52, -12, 1, -12, -9, -14, 42, 55, 18, 37, 28, 42, 15, 62);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(13, -12, -28, 3, -22, 13, 8, -18, 6, 3, -4, 9, 3, -38, -19, 11);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-12, 17, 30, -6, 24, -11, -6, 17, -3, -4, 5, -12, -2, 36, 20, -10);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 2;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-25, 15, -68, -3, 5, 6, 0, 15, -36, -60, -80, -53, -34, -65, -8, -48);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(21, -14, 68, 5, -1, -9, -1, -17, 33, 60, 77, 54, 41, 65, 10, 51);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-2, -31, -31, 3, -28, 0, 3, -27, 6, -2, -5, 20, 1, -42, -7, 11);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-1, 32, 37, -2, 26, -2, -7, 28, -4, 2, 3, -19, -6, 42, 11, -15);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 3;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-11, 31, -7, 11, 7, 2, 6, 5, -61, -88, -122, -30, -48, -73, -46, -51);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(10, -34, 9, -12, -1, -1, -5, -2, 57, 92, 127, 28, 51, 72, 43, 46);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-13, 8, 3, -1, -2, 3, 6, 11, 0, -2, 9, 17, 4, -36, -7, -6);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(17, -7, -5, 1, 8, 0, -9, -13, -5, 2, -4, -11, -2, 38, 10, 7);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_1 = ix + 2;
			x_2 = jx + 0;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-10, 11, -27, 7, 1, -2, 5, 11, -53, -46, -37, -85, -46, -15, 6, -52);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(5, -11, 26, -2, -4, 0, -5, -6, 54, 45, 36, 91, 46, 9, -9, 49);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(19, 11, -36, 14, -63, 20, 7, 0, 1, 4, 13, 0, 2, -66, -7, -5);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-15, -11, 40, -14, 59, -16, -4, 2, -2, -7, -16, -2, 0, 67, 7, 1);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-15, 26, -42, 10, 2, 16, 6, 9, -32, -24, -42, -66, -49, -60, 4, -75);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(10, -28, 43, -14, -1, -15, -5, -9, 36, 25, 46, 64, 43, 58, -2, 75);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(36, -24, -38, 13, -33, 12, 4, -17, 9, 8, 25, 1, 12, -39, -44, 1);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-34, 20, 40, -12, 34, -10, -6, 16, -2, -8, -27, -5, -13, 41, 45, -6);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 2;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-3, 21, -18, -4, 6, 17, 8, 13, -30, -44, -79, -51, -54, -63, 7, -51);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(2, -19, 23, 7, -2, -16, -4, -7, 33, 44, 79, 51, 56, 65, -5, 54);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(24, -21, -30, 18, -7, 10, 7, 16, 10, 5, 13, 12, 7, -29, -35, 19);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-24, 22, 29, -19, 12, -11, -4, -17, -11, 0, -13, -10, -6, 29, 38, -21);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 3;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-23, 11, -71, 1, 2, -1, 7, 14, -50, -74, -115, -29, -62, -60, -18, -44);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(20, -13, 76, -4, -6, 1, -9, -15, 57, 70, 122, 31, 61, 58, 16, 43);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-17, 7, 1, 3, 8, 12, 13, 12, 11, 3, 0, 15, 12, -40, 6, 7);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(18, -4, -4, -9, -8, -14, -7, -10, -9, -6, -4, -13, -9, 44, -3, -12);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_1 = ix + 3;
			x_2 = jx + 0;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(1, 4, -31, 14, 4, -2, 1, 6, -31, -50, -46, -87, 4, 4, -1, -25);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(2, -3, 33, -9, -7, 4, 3, -6, 30, 49, 46, 87, -4, -2, 8, 25);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(28, 11, -31, 9, -7, 5, 2, 12, 1, 3, -24, -15, -9, -88, 5, -6);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-24, -9, 29, -8, 7, -7, -6, -14, -7, -4, 24, 13, 9, 88, -3, 11);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-13, 10, -41, 0, 6, 11, 8, 0, -36, -61, -31, -17, 8, -72, 10, -30);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(11, -12, 41, -5, -7, -13, -4, 4, 42, 57, 29, 17, -10, 73, -13, 32);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(19, -24, -32, 17, 18, 4, -2, 3, 30, 19, 27, -5, 24, -47, 2, 3);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-17, 24, 29, -14, -17, -4, -1, -7, -29, -15, -22, 8, -21, 48, 3, 2);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 2;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(23, 21, -32, 2, 4, 17, 5, 6, -43, -51, -35, -22, 4, -83, 17, -30);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-23, -18, 32, 2, 1, -12, -3, -3, 43, 57, 34, 27, -2, 83, -18, 25);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(13, -44, -47, 10, 9, 7, -1, 14, 16, 16, 29, 4, 6, -37, -17, 5);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-11, 40, 47, -8, -14, -7, 3, -11, -16, -17, -30, -10, -8, 42, 20, -5);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
			x_2 = jx + 3;

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(-20, 21, 12, -2, 0, -8, 4, 10, -67, -54, -77, -20, 5, -80, -26, -17);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
			qw = _mm_set_epi8(25, -21, -11, 5, -2, 6, 0, -13, 64, 58, 82, 15, -5, 81, 29, 19);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(-28, 8, -1, 0, 4, 7, 3, 0, 11, 6, 23, 18, 17, -31, -9, 17);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][16]);
			qw = _mm_set_epi8(26, -6, -4, 2, -2, -9, -6, -2, -6, -6, -23, -15, -16, 32, 10, -22);
			qx = _mm_maddubs_epi16(qx, qw);
			cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);
		}
	}

	for (int i = 0; i < 1; i++)
	    for (int j = 0; j < 1; j++)
	        for (int k = 0; k < 2; k++){
	            qx = cx5[i][j][k];              
	            lo = _mm_srai_epi32(_mm_unpacklo_epi16(qx, qx), 16);
	            hi = _mm_srai_epi32(_mm_unpackhi_epi16(qx, qx), 16);
	            sum1 = _mm_hadd_epi32(hi, lo);
	            sum2 = _mm_hadd_epi32(sum1, sum1);
		        _mm_store_si128((__m128i*)res, sum2);
	            x5[i][j][k] += (res[0] + res[1]) * 0.02773932396896242f * 0.020375443622469902f;
	        }
	static float x6[1][1][2] = {0};
	static float max6 = x5[0][0][0] > x5[0][0][1] ? x5[0][0][0] : x5[0][0][1];
	x6[0][0][0] = (float)exp(x5[0][0][0] - max6);
	x6[0][0][1] = (float)exp(x5[0][0][1] - max6);
	static float sum6;
	sum6 = x6[0][0][0] + x6[0][0][1];
	x6[0][0][0] /= sum6;
	x6[0][0][1] /= sum6;
	scores[0] = x6[0][0][0];
	scores[1] = x6[0][0][1];
	return;
}

#ifdef CNN_TEST
#include <stdio.h>
#ifdef TIMING
#include <ctime>
#endif
    
int main()
{
    int i, j, k, res, width, height, max_colour;
    unsigned char byte;
    float x[16 * 16 * 1];
    float scores[2];
    FILE *f = fopen("img.pgm", "r");
    fscanf (f, "P5\n%d %d\n%d\n", &width, &height, &max_colour);
    for (j = 0; j < 16; j++)
        for (i = 0; i < 16; i++)
            for (k = 0; k < 1; k++)
            {
                fread(&byte, sizeof(unsigned char), 1, f);
                x[j * 16 * 1 + i * 1 + k] = byte / 255.f;
            }
    fclose(f);
    res = 0;
#ifdef TIMING
    clock_t begin = clock();
	for (i = 0; i < TIMING; i++)  
		cnn_qBallCustomLoss(x, scores);
	 clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	printf("%f %f, %f s ", scores[0], scores[1], elapsed_secs);
#else
    cnn_qBallCustomLoss(x, scores);
#endif
    return scores[1] > scores[0];
}
#endif

#if defined(__clang__)
# pragma clang diagnostic pop
#endif

#if defined(_MSC_VER)
# pragma warning(pop)
#endif
