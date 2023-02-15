
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
void cnn(float *x_in, float *scores)
{
	__m128 w, x, y, y2, t, t2;
unsigned char buf alignas(16) [16];
__m128i qw, qx, qt1, qt2, lo, hi, sum1, sum2, sum3;
int res alignas(16) [4];
	float x0[16][16][1];
	for (int xi = 0; xi < 16; xi += 1)
	{
	x0[xi][0][0] = x_in[xi * 16 * 1 + 0 * 1 + 0] - 0.5836217875631101f;
	x0[xi][1][0] = x_in[xi * 16 * 1 + 1 * 1 + 0] - 0.5836217875631101f;
	x0[xi][2][0] = x_in[xi * 16 * 1 + 2 * 1 + 0] - 0.5836217875631101f;
	x0[xi][3][0] = x_in[xi * 16 * 1 + 3 * 1 + 0] - 0.5836217875631101f;
	x0[xi][4][0] = x_in[xi * 16 * 1 + 4 * 1 + 0] - 0.5836217875631101f;
	x0[xi][5][0] = x_in[xi * 16 * 1 + 5 * 1 + 0] - 0.5836217875631101f;
	x0[xi][6][0] = x_in[xi * 16 * 1 + 6 * 1 + 0] - 0.5836217875631101f;
	x0[xi][7][0] = x_in[xi * 16 * 1 + 7 * 1 + 0] - 0.5836217875631101f;
	x0[xi][8][0] = x_in[xi * 16 * 1 + 8 * 1 + 0] - 0.5836217875631101f;
	x0[xi][9][0] = x_in[xi * 16 * 1 + 9 * 1 + 0] - 0.5836217875631101f;
	x0[xi][10][0] = x_in[xi * 16 * 1 + 10 * 1 + 0] - 0.5836217875631101f;
	x0[xi][11][0] = x_in[xi * 16 * 1 + 11 * 1 + 0] - 0.5836217875631101f;
	x0[xi][12][0] = x_in[xi * 16 * 1 + 12 * 1 + 0] - 0.5836217875631101f;
	x0[xi][13][0] = x_in[xi * 16 * 1 + 13 * 1 + 0] - 0.5836217875631101f;
	x0[xi][14][0] = x_in[xi * 16 * 1 + 14 * 1 + 0] - 0.5836217875631101f;
	x0[xi][15][0] = x_in[xi * 16 * 1 + 15 * 1 + 0] - 0.5836217875631101f;
	}
	static float x1 alignas(16) [16][16][8] = {0};
	for (int i = 0; i < 16; i += 1)
	{
		for (int j = 0; j < 16; j += 1)
		{
			x1[i][j][0] = 0.03989183157682419f;
			x1[i][j][1] = -0.008467871695756912f;
			x1[i][j][2] = -0.18999922275543213f;
			x1[i][j][3] = -0.14836639165878296f;
			x1[i][j][4] = 0.0015352556947618723f;
			x1[i][j][5] = -0.2954272925853729f;
			x1[i][j][6] = 0.039320070296525955f;
			x1[i][j][7] = 0.038555026054382324f;
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

					w = _mm_set_ps(0.1560819447040558f, 0.30243250727653503f, -0.43852144479751587f, -0.3703727722167969f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.54542076587677f, -0.08736631274223328f, 0.0029384249355643988f, 0.3326971232891083f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.6507819890975952f, 0.3622538149356842f, 0.3723638951778412f, -0.4369346499443054f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.13425010442733765f, -0.4624957740306854f, -0.13564905524253845f, 0.3572583794593811f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.925951361656189f, 0.08520200848579407f, 0.371324747800827f, -0.3110533058643341f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.35369712114334106f, -0.0738290548324585f, 0.2656768560409546f, -0.7147999405860901f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
			}
			x_1 = ix + 1;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.3508879244327545f, -0.06031960994005203f, -0.6392368674278259f, 0.19072116911411285f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.1667976677417755f, -0.15861085057258606f, -0.42946067452430725f, -0.058149661868810654f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.5886675715446472f, -0.0020013723988085985f, 0.09101161360740662f, -0.3174373209476471f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.6352745890617371f, -0.5341864824295044f, 0.06410403549671173f, 0.5648777484893799f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.4009113013744354f, 0.26625335216522217f, 0.11350692808628082f, 0.3182257413864136f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.2161913365125656f, 0.10756710171699524f, -0.02235664613544941f, -0.22382225096225739f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
			}
			x_1 = ix + 2;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.20388172566890717f, 0.033551234751939774f, -0.4759614169597626f, 0.8014712929725647f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.3713614344596863f, -0.11489430069923401f, -0.48890209197998047f, -0.3817545473575592f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.4109117388725281f, -0.06568379700183868f, 0.21143589913845062f, 0.32579106092453003f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.020787546411156654f, 0.6095505356788635f, 0.08413373678922653f, 0.4014896750450134f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.5528272986412048f, 0.16048330068588257f, 0.4478694796562195f, -0.217317596077919f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.24278390407562256f, 0.6894845366477966f, -0.18674133718013763f, -0.06577053666114807f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
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
		}
	}
	static float x2[8][8][8] = {0};
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
		}
	}
	static float x3 alignas(16) [8][8][16] = {0};
	for (int i = 0; i < 8; i += 1)
	{
		for (int j = 0; j < 8; j += 1)
		{
			x3[i][j][0] = 0.08651088178157806f;
			x3[i][j][1] = 0.09490945935249329f;
			x3[i][j][2] = 0.01003597304224968f;
			x3[i][j][3] = 0.23136308789253235f;
			x3[i][j][4] = -0.12283578515052795f;
			x3[i][j][5] = -0.03149667754769325f;
			x3[i][j][6] = -0.15073588490486145f;
			x3[i][j][7] = -0.025728993117809296f;
			x3[i][j][8] = -0.002652120543643832f;
			x3[i][j][9] = -0.06298455595970154f;
			x3[i][j][10] = 0.13163338601589203f;
			x3[i][j][11] = -0.005559318698942661f;
			x3[i][j][12] = -0.1265489012002945f;
			x3[i][j][13] = -0.15474848449230194f;
			x3[i][j][14] = -0.11072861403226852f;
			x3[i][j][15] = 0.005210805218666792f;
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

					w = _mm_set_ps(0.10687340795993805f, -0.058296944946050644f, 0.32945898175239563f, -0.10283394157886505f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.146449476480484f, -0.25718632340431213f, -0.08190225064754486f, -0.21550504863262177f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.13155341148376465f, 0.17007127404212952f, -0.32332029938697815f, -0.14937013387680054f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.07023859769105911f, -0.1010223999619484f, 0.07791943103075027f, 0.024713698774576187f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.1977846622467041f, 0.4805377423763275f, 0.2863008379936218f, -0.08292197436094284f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1326894313097f, -0.04168292135000229f, 0.12111884355545044f, -0.1455967128276825f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.09153425693511963f, 0.5130406618118286f, -0.26309385895729065f, -0.23268702626228333f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.0038553725462406874f, -0.41278302669525146f, 0.14698182046413422f, 0.05493175610899925f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.04270368069410324f, 0.05113910511136055f, 0.14282897114753723f, 0.21396812796592712f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.05603397265076637f, -0.07349028438329697f, 0.008103909902274609f, 0.07821640372276306f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.016074422746896744f, 0.07193130999803543f, -0.12136738002300262f, 0.11421453207731247f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.0260283425450325f, -0.21600337326526642f, -0.10646702349185944f, -0.1477256864309311f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.3033449351787567f, -0.12741252779960632f, 0.0315132811665535f, -0.3026115596294403f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.07023096084594727f, -0.2182220071554184f, 0.09436889737844467f, 0.30588066577911377f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.047464385628700256f, 0.29651665687561035f, 0.5118600130081177f, 0.03133327513933182f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.19130118191242218f, 0.48457401990890503f, 0.30959492921829224f, -0.2774398922920227f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.5597813725471497f, 0.5598654747009277f, 0.23563015460968018f, -0.06931905448436737f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.07317258417606354f, -0.05808689445257187f, -0.3712310194969177f, -0.24473132193088531f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.003926509991288185f, 0.1288420855998993f, -0.1361663043498993f, -0.05879845470190048f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.09757447987794876f, -0.6060186624526978f, 0.72847980260849f, 0.0697999969124794f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.13122230768203735f, -0.12403779476881027f, -0.08571428060531616f, 0.2092105746269226f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1194467842578888f, -0.06299447268247604f, 0.09388178586959839f, -0.24037738144397736f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.10157951712608337f, 0.20454180240631104f, 0.09613462537527084f, 0.0038925884291529655f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.12783657014369965f, 0.15522286295890808f, 0.015023961663246155f, 0.14372684061527252f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.23179663717746735f, -0.3292667865753174f, 0.003296215320006013f, 0.1276187300682068f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.0947979986667633f, -0.12013965845108032f, 0.06121956557035446f, -0.14678849279880524f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.030326951295137405f, 0.07210967689752579f, 0.3032795786857605f, -0.39292970299720764f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.10696546733379364f, 0.3127264678478241f, -0.007460157852619886f, 0.14069189131259918f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.20739208161830902f, -0.20958727598190308f, -0.2577802538871765f, -0.14479784667491913f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.2831464111804962f, -0.05529861897230148f, 0.0748358890414238f, -0.06601808965206146f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.10538201779127121f, 0.0960020124912262f, -0.10331518203020096f, -0.133653461933136f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.006452738307416439f, -0.07897835969924927f, 0.006198056973516941f, -0.04053381457924843f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(0.07899992167949677f, -0.2978339195251465f, -0.24236555397510529f, -0.0717242956161499f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.011975516565144062f, -0.183562234044075f, -0.08951421082019806f, 0.029929254204034805f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.009841314516961575f, 0.09304986149072647f, -0.03343803063035011f, -0.07209936529397964f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.06863285601139069f, 0.34265369176864624f, -0.04473152756690979f, -0.1462901383638382f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.18261370062828064f, 0.23115447163581848f, -0.39350274205207825f, -0.008819754235446453f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.2575925886631012f, 0.016800545156002045f, 0.1890587955713272f, -0.2613753378391266f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.08350920677185059f, 0.5107959508895874f, -0.171921506524086f, -0.6879701614379883f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.130418598651886f, -0.18061910569667816f, 0.2399635761976242f, 0.09727609902620316f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.1659347414970398f, -0.1470729261636734f, 0.16166041791439056f, 0.08178504556417465f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.06426237523555756f, -0.07732716202735901f, 0.34543272852897644f, 0.04120141640305519f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.3707520067691803f, -0.0030483680311590433f, 0.37604236602783203f, 0.0042112949304282665f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.15391851961612701f, -0.2912517189979553f, 0.18735070526599884f, -0.17557376623153687f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-5.481320840772241e-06f, 0.022273847833275795f, 0.5403451919555664f, 0.05103931576013565f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.03456571325659752f, -0.15169326961040497f, 0.20574559271335602f, 0.3321629762649536f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.125657320022583f, -0.45382213592529297f, 0.44880783557891846f, 0.24780631065368652f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.4118503928184509f, 0.1521180123090744f, 0.1006959080696106f, -0.22741764783859253f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.1332976222038269f, 0.08969419449567795f, 0.08899862319231033f, -0.007530201226472855f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.08912894129753113f, -0.30308544635772705f, -0.21316330134868622f, 0.20517081022262573f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.060365378856658936f, 0.10299227386713028f, 0.2575347423553467f, -0.03764236718416214f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.19534015655517578f, -0.5670735239982605f, 0.10452169924974442f, -0.238506481051445f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.17211352288722992f, 0.09604810178279877f, -0.5859155654907227f, 0.0058422996662557125f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.22810375690460205f, -0.2508411109447479f, 0.33659273386001587f, 0.3443739414215088f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.2629607319831848f, 0.2812824249267578f, -0.02051261067390442f, 0.041735634207725525f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.10756278038024902f, -0.15128563344478607f, 0.12382588535547256f, 0.13270941376686096f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.17148476839065552f, 0.09798241406679153f, -0.1579180508852005f, -0.13972419500350952f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.16966620087623596f, 0.06949099898338318f, -0.27286210656166077f, -0.2720070481300354f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.17387712001800537f, 0.2923751771450043f, -0.079447440803051f, 0.1279493123292923f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.22501251101493835f, 0.18246649205684662f, -0.1272745430469513f, -0.03511103242635727f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.2425801306962967f, -0.0695796087384224f, -0.28872406482696533f, -0.2928803563117981f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.13189548254013062f, 0.11234359443187714f, -0.23456904292106628f, 0.1311943382024765f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.03013748675584793f, -0.1728488653898239f, -0.3337652385234833f, 0.08817246556282043f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.3197459876537323f, -0.27065223455429077f, -0.08445369452238083f, -0.21230320632457733f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(-0.14519979059696198f, 0.07361891865730286f, 0.00983502622693777f, -0.0658319741487503f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1526997983455658f, 0.17911382019519806f, 0.0733879953622818f, 0.14310266077518463f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.022724898532032967f, -0.5296104550361633f, 0.4319680631160736f, 0.08184836059808731f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.2978244423866272f, 0.572534441947937f, 0.06319506466388702f, 0.03407164290547371f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.13949501514434814f, -0.004667388275265694f, -0.3882911503314972f, -0.1597478836774826f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.31719136238098145f, -0.07424166798591614f, -0.07347000390291214f, 0.07323095947504044f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.1652149260044098f, 0.24872824549674988f, -0.20833872258663177f, -0.07423831522464752f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.1437174379825592f, -0.45002642273902893f, 0.7361864447593689f, -0.04935178533196449f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.11917142570018768f, 0.0767025500535965f, -0.03649011254310608f, 0.1661582738161087f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.10925911366939545f, 0.12648780643939972f, 0.037029776722192764f, 0.10246488451957703f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.2999102473258972f, 0.018520597368478775f, 0.004670627415180206f, 0.08972707390785217f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.2510649859905243f, -0.10163722932338715f, 0.10755668580532074f, -0.0859924703836441f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.11017400026321411f, 0.3794000446796417f, 0.3378308117389679f, -0.24612221121788025f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.16395826637744904f, -0.08751457184553146f, -0.2130996286869049f, -0.4042758345603943f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.054723069071769714f, -0.014864624477922916f, 0.3331768214702606f, -0.020011071115732193f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.4007246196269989f, 0.1947273313999176f, 0.40436288714408875f, -0.393839567899704f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.1331055611371994f, 0.049108896404504776f, -0.049709104001522064f, 0.14752142131328583f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.24225252866744995f, -0.03256203979253769f, 0.2960832715034485f, 0.06449490040540695f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.25925686955451965f, 0.37162041664123535f, 0.03552106395363808f, -0.030468355864286423f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.08992181718349457f, -0.23353314399719238f, 0.545676052570343f, -0.18507499992847443f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.26909971237182617f, -0.062130119651556015f, -0.27480337023735046f, 0.2247695028781891f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.4404756426811218f, 0.26589253544807434f, -0.013384816236793995f, 0.1504514217376709f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.17826628684997559f, -0.22176286578178406f, -0.5482487678527832f, 0.1803223341703415f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.1726437360048294f, 0.07830040901899338f, -0.02736469730734825f, 0.5179409384727478f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.11408679932355881f, 0.08879569917917252f, -0.4689188003540039f, 0.06072920188307762f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.21770858764648438f, 0.4804311692714691f, -0.4652608633041382f, -0.08034893125295639f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.03404565900564194f, 0.08731342852115631f, -0.019803514704108238f, -0.03893135115504265f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.09863968193531036f, 0.07870432734489441f, -0.06974942237138748f, 0.09398657828569412f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.25881659984588623f, -0.2024746835231781f, 0.040182217955589294f, -0.006749029736965895f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.08670001477003098f, 0.1665455549955368f, -0.08211402595043182f, -0.08411546051502228f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.21920211613178253f, -0.3024623692035675f, -0.4003678262233734f, 0.05366241931915283f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.14242501556873322f, 0.13250739872455597f, -0.035011906176805496f, -0.44544580578804016f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
			}
			x_1 = ix + 1;
			if (x_1 >= 0 && x_1 < 8)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(0.2877446711063385f, 0.35113009810447693f, -0.7069751024246216f, 0.057358480989933014f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.08203501999378204f, -0.08903856575489044f, 0.1122732013463974f, -0.21276487410068512f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.2272552251815796f, 0.43611782789230347f, -0.12640035152435303f, -0.3388901352882385f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.20593547821044922f, 0.06893768906593323f, -0.0049386522732675076f, -0.12056111544370651f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.12896162271499634f, 0.19279903173446655f, -0.28836706280708313f, 0.07912525534629822f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.41766029596328735f, -0.07581992447376251f, -0.010279752314090729f, 0.04869132116436958f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.1830761879682541f, 0.37201741337776184f, -0.13574081659317017f, 0.1815854012966156f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.08875139057636261f, 0.12489818036556244f, 0.03505956381559372f, 0.24426555633544922f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.3497786521911621f, -0.3124612867832184f, 0.4330034852027893f, 0.1953878253698349f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1293001025915146f, -0.33711814880371094f, -0.13246500492095947f, 0.14307266473770142f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.06376715004444122f, -0.10279195010662079f, 0.06933727860450745f, 0.197758287191391f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.060627665370702744f, -0.13224953413009644f, -0.1683131605386734f, 0.2079884111881256f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.14587029814720154f, 0.338001549243927f, 0.0009458503918722272f, 0.18319763243198395f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.00696936110034585f, -0.2236853986978531f, 0.10076858103275299f, 0.20068024098873138f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.0745517760515213f, -0.3453547954559326f, 0.17162559926509857f, -0.37070590257644653f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.37570807337760925f, -0.06306731700897217f, -0.08875724673271179f, -0.11842073500156403f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.12599211931228638f, -0.04264036566019058f, 0.3772829473018646f, 0.28891846537590027f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.05472628027200699f, 0.015765586867928505f, -0.056745659559965134f, 0.0267216507345438f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.2875197231769562f, 0.02423490211367607f, -0.08292828500270844f, 0.23316195607185364f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.1519017219543457f, -0.5882971286773682f, 0.561619222164154f, -0.05901557579636574f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.002783292904496193f, 0.20833082497119904f, -0.20178142189979553f, 0.07977975159883499f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.5300347208976746f, -0.3186652958393097f, -0.16335716843605042f, 0.17425285279750824f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.18845386803150177f, -0.03773529827594757f, 0.26145219802856445f, -0.3440852761268616f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.012252699583768845f, -0.039875783026218414f, -0.3533850610256195f, 0.2617364525794983f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.4152701497077942f, -0.11323990672826767f, -0.9043914079666138f, 0.11237261444330215f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.11080111563205719f, -0.1454911082983017f, 0.14754192531108856f, -0.1382322609424591f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.08421481400728226f, 0.44379258155822754f, -0.23175236582756042f, -0.2228393256664276f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.11303006857633591f, 0.4104328155517578f, -0.18687254190444946f, -0.06644757837057114f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.05090589448809624f, 0.22481854259967804f, -0.32333502173423767f, 0.060484666377305984f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.2295852154493332f, 0.12224065512418747f, 0.002278154017403722f, 0.17845100164413452f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.012743115425109863f, 0.06878282129764557f, 0.00450978334993124f, 0.01440825778990984f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.06512884050607681f, 0.03874947130680084f, 0.3167133927345276f, 0.33877822756767273f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(0.07289133220911026f, 0.1951460987329483f, -0.4931371510028839f, 0.06390203535556793f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.2999174892902374f, 0.09580357372760773f, -0.06546629965305328f, 0.2675240933895111f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.15684568881988525f, -0.35537514090538025f, -0.25057870149612427f, 0.21909603476524353f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.23226943612098694f, -0.183884397149086f, 0.1978818029165268f, -0.08056443184614182f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.40151262283325195f, 0.029230749234557152f, -0.5818419456481934f, -0.3524353504180908f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.3658866286277771f, 0.10715338587760925f, 0.20358523726463318f, 0.4564276337623596f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.23242241144180298f, -0.30769091844558716f, -0.01677919551730156f, -0.2940857708454132f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.0654442310333252f, 0.1536879539489746f, -0.13042519986629486f, 0.3145158588886261f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.678382396697998f, -0.07100944221019745f, 0.044008441269397736f, 0.2529658079147339f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.18911927938461304f, -0.32504308223724365f, 0.057058677077293396f, 0.03428719937801361f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.1183243840932846f, 0.10832171142101288f, -0.11250880360603333f, -0.2847999632358551f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.08271133154630661f, 0.12919628620147705f, -0.3471267521381378f, 0.06117267906665802f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.3303188681602478f, 0.20627030730247498f, 0.24288754165172577f, 0.25483590364456177f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.3356725871562958f, 0.19670766592025757f, 0.12393283098936081f, -0.07745978236198425f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.12513577938079834f, -0.33779674768447876f, -0.017888346686959267f, 0.1272241473197937f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.004750097170472145f, 0.460114061832428f, -0.1785023808479309f, -0.22959503531455994f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.5186320543289185f, -0.36628982424736023f, -0.051928695291280746f, 0.6238738298416138f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.11239687353372574f, -0.40032878518104553f, -0.6031194925308228f, 0.21161654591560364f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.009492004290223122f, -0.5006470680236816f, -0.23661291599273682f, 0.05265086144208908f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.1723514199256897f, 0.07514767348766327f, -0.02948594093322754f, -0.1557552069425583f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.14834024012088776f, -0.004224749282002449f, 0.030673597007989883f, -0.10964362323284149f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.40974077582359314f, -0.4424160420894623f, 0.3521724343299866f, 0.4542548954486847f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.026861095800995827f, -0.17345398664474487f, -0.016638098284602165f, 0.13025011122226715f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.07678971439599991f, -0.28080472350120544f, -0.33299678564071655f, 0.07582497596740723f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.2125108242034912f, 0.14841973781585693f, -0.6839345693588257f, -0.011455041356384754f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1472344845533371f, 0.025791343301534653f, 0.4566913843154907f, 0.32295048236846924f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.015240897424519062f, -0.09087882190942764f, 0.03741282597184181f, 0.21789546310901642f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.17378747463226318f, -0.005647636484354734f, 0.039224009960889816f, 0.07600179314613342f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.22230613231658936f, 0.26695653796195984f, 0.17301048338413239f, -0.05219268798828125f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.34219032526016235f, -0.06833644211292267f, 0.01991586573421955f, -0.051799047738313675f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.16357813775539398f, -0.28506702184677124f, 0.1751004159450531f, 0.1855596899986267f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.3136530816555023f, -0.15610583126544952f, 0.024529606103897095f, -0.41729047894477844f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(-0.16468386352062225f, -0.03635610640048981f, 0.09588976949453354f, 0.08183491230010986f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.15119995176792145f, 0.14944632351398468f, -0.7973917126655579f, -0.10503941774368286f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.11895778030157089f, -0.731391966342926f, -0.1830531656742096f, 0.2703014314174652f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.18510891497135162f, 0.11545535922050476f, -0.02858494035899639f, 0.3372533321380615f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.08942239731550217f, -0.40847161412239075f, 0.27480438351631165f, -0.021301863715052605f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.395054429769516f, -0.3200167417526245f, 0.4222397804260254f, 0.3696695864200592f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.08578158169984818f, -0.9765959978103638f, 0.35168126225471497f, 0.27270305156707764f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.3457196056842804f, -0.11468828469514847f, 0.3602878749370575f, 0.3682081699371338f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.5086663961410522f, -0.13187773525714874f, -0.17257167398929596f, -0.16361436247825623f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.10109866410493851f, 0.21335558593273163f, 0.3122571110725403f, 0.07285632938146591f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.29328829050064087f, -0.16254238784313202f, 0.2452671378850937f, 0.036851074546575546f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.04690982401371002f, 0.0629122331738472f, -0.15727533400058746f, 0.01270989142358303f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.0423598438501358f, 0.1213034838438034f, 0.1363976150751114f, -0.19683793187141418f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.5405077338218689f, 0.28616809844970703f, -0.26998013257980347f, -0.42087462544441223f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.06279010325670242f, -0.08489328622817993f, 0.196298748254776f, -0.02599567361176014f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.48138079047203064f, 0.16342557966709137f, -0.0088143115863204f, -0.21572580933570862f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.4676533341407776f, -0.03372250497341156f, -0.09265252947807312f, -0.1359967440366745f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.10453393310308456f, -0.2898874282836914f, 0.03133973851799965f, 0.0497739240527153f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.15519185364246368f, -0.017346447333693504f, 0.2905311584472656f, -0.031108776107430458f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.14451555907726288f, 0.26455622911453247f, 0.1549191176891327f, 0.08931317925453186f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.45344188809394836f, -0.31529030203819275f, 0.5522900223731995f, -0.008333045057952404f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.37250852584838867f, 0.09017061442136765f, 0.12032066285610199f, 0.21441157162189484f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.3503815829753876f, -0.44091498851776123f, -0.13509142398834229f, 0.36320698261260986f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.30709108710289f, 0.15405689179897308f, 0.0660487711429596f, -0.39392584562301636f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.17785164713859558f, 0.12079163640737534f, -0.1157047301530838f, -0.134905606508255f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.043679650872945786f, 0.2411716729402542f, -0.4830351173877716f, 0.15058042109012604f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.13016818463802338f, -0.2745964527130127f, -0.047692254185676575f, 0.14906573295593262f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.13259173929691315f, -0.030993174761533737f, -0.005766190588474274f, 0.15918336808681488f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.13632100820541382f, -0.18143263459205627f, 0.28248706459999084f, -0.04252978041768074f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.050458356738090515f, 0.21041683852672577f, -0.25413092970848083f, -0.021384157240390778f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.2872064709663391f, -0.2625032067298889f, 0.19699281454086304f, 0.20929847657680511f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.36227893829345703f, 0.21029821038246155f, -0.0635301023721695f, -0.5972326397895813f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
			}
			x_1 = ix + 2;
			if (x_1 >= 0 && x_1 < 8)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(0.22959041595458984f, -0.020420312881469727f, -0.19704526662826538f, 0.23427177965641022f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.5219101905822754f, -0.08971643447875977f, 0.13231948018074036f, 0.0003332195628900081f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.11153283715248108f, -0.17702341079711914f, 0.08537035435438156f, -0.3303142189979553f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.3116356134414673f, 0.04620153829455376f, -0.009639565832912922f, -0.06520277261734009f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.26208260655403137f, 0.08877243846654892f, -0.096864253282547f, 0.13816826045513153f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.08152066171169281f, -0.5840874314308167f, -0.08541315048933029f, 0.18979015946388245f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.008991530165076256f, -0.13012298941612244f, -0.020642707124352455f, 0.226694718003273f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.13213136792182922f, 0.22127272188663483f, 0.10910959541797638f, 0.23324623703956604f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.05371003970503807f, -0.2778397500514984f, 0.19022443890571594f, 0.09099429845809937f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.2742950916290283f, -0.2216702103614807f, 0.11309541016817093f, 0.07033612579107285f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.010077744722366333f, -0.1071627289056778f, 0.1673835963010788f, 0.1446281224489212f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.11323701590299606f, 0.006644811015576124f, 0.009334447793662548f, 0.2308778315782547f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.23135103285312653f, -0.030141424387693405f, 0.443774938583374f, -0.06688421964645386f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.42721372842788696f, -0.28062471747398376f, 0.07787645608186722f, 0.20146799087524414f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.049755752086639404f, -0.284952312707901f, -0.38046205043792725f, -0.19598707556724548f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.2937036156654358f, 0.041561126708984375f, 0.06802797317504883f, -0.13188670575618744f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.222473606467247f, -0.14449374377727509f, 0.20047803223133087f, 0.4147510528564453f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.5625787973403931f, -0.4030669927597046f, 0.1769506335258484f, -0.06504551321268082f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.19263912737369537f, -0.5572090148925781f, -0.011371881701052189f, 0.10513976216316223f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.16778208315372467f, 0.3579760193824768f, -0.015888728201389313f, 0.10576054453849792f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.2268827110528946f, 0.29660525918006897f, 0.5211476683616638f, -0.30783843994140625f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.13494332134723663f, 0.1349661499261856f, -0.19147342443466187f, 0.09951211512088776f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.20071323215961456f, 0.18969638645648956f, 0.4341139495372772f, -0.07722111791372299f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.1540394276380539f, 0.053788624703884125f, 0.1735992580652237f, 0.10009419173002243f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.18283648788928986f, 0.022894885390996933f, -0.130732461810112f, -0.02091897651553154f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.33359044790267944f, 0.06044084578752518f, -0.15414994955062866f, 0.06986074149608612f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.0760076567530632f, 0.08506172895431519f, 0.02635161206126213f, -0.9087573885917664f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.48437902331352234f, 0.0971187949180603f, -0.6093946695327759f, 0.06178231164813042f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.1066896840929985f, 0.33549898862838745f, 0.21905426681041718f, -0.18793487548828125f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.36541473865509033f, 0.25934088230133057f, -0.04650832712650299f, 0.29242968559265137f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.3982102870941162f, 0.052666958421468735f, -0.1378905028104782f, -0.2076231837272644f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.2188456803560257f, 0.04049278795719147f, -0.21317356824874878f, 0.11244441568851471f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(-0.004967415239661932f, -0.007899990305304527f, -0.354071706533432f, 0.20473116636276245f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.47615712881088257f, -0.07995577901601791f, 0.268680602312088f, 0.14902962744235992f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.1935790330171585f, -0.4312198758125305f, 0.30047687888145447f, 0.08725297451019287f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.2355712503194809f, 0.04599907994270325f, 0.5941383838653564f, -0.1677115112543106f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.17987869679927826f, 0.31056293845176697f, -0.2602320611476898f, 0.0905207097530365f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.04719997197389603f, -0.2595682740211487f, -0.27188095450401306f, -0.02081926539540291f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.17408566176891327f, -0.5226640105247498f, 0.04394643381237984f, -0.5017198920249939f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.2824529707431793f, 0.09754644334316254f, -0.3697785437107086f, 0.29655519127845764f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.17196263372898102f, -0.05807117000222206f, -0.12749284505844116f, 0.18701373040676117f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.0312734991312027f, -0.23383191227912903f, -0.19131837785243988f, 0.10934773832559586f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.025372199714183807f, 0.05020938813686371f, -0.01974024996161461f, -0.002483569784089923f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.0030105519108474255f, 0.0995442196726799f, -0.06746522337198257f, 0.04574776440858841f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.37076741456985474f, -0.2165071964263916f, 0.027941158041357994f, 0.07543039321899414f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.03837522864341736f, 0.3243536949157715f, 0.02930678240954876f, -0.26965755224227905f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.3509955406188965f, -0.06476389616727829f, -0.031261593103408813f, 0.036494333297014236f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.3493983745574951f, 0.27089419960975647f, -0.09133804589509964f, 0.025817835703492165f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.15679578483104706f, -0.13194842636585236f, -0.31307414174079895f, 0.3417714536190033f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.02841790020465851f, -0.2080477625131607f, -0.35467877984046936f, 0.13309389352798462f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.1817714273929596f, 0.11617480218410492f, -0.16363684833049774f, 0.07057438790798187f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.2125016450881958f, -0.14783506095409393f, 0.03517678380012512f, 0.2643059492111206f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.328946977853775f, 0.22943466901779175f, 0.30809175968170166f, 0.0032246611081063747f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.3425988554954529f, -0.1661766767501831f, 0.15942710638046265f, -0.03832248970866203f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.09003069996833801f, 0.03238154575228691f, 0.21672281622886658f, -0.22910508513450623f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.24987760186195374f, 0.041050221771001816f, -0.5581569671630859f, 0.039289336651563644f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.262222558259964f, 0.07664912939071655f, -0.17657256126403809f, 0.06050759181380272f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.3769975006580353f, -0.015891212970018387f, 0.26792827248573303f, 0.18717721104621887f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.024981409311294556f, 0.09995793551206589f, 0.13897156715393066f, -0.0915520042181015f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.8553535342216492f, 0.03955911844968796f, -0.0645175576210022f, 0.08572272956371307f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.1320718675851822f, 0.07139110565185547f, 0.20281153917312622f, -0.16614358127117157f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.026282649487257004f, 0.18778981268405914f, 0.12356673926115036f, -0.20320428907871246f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.3413887321949005f, 0.24244479835033417f, -0.4285905063152313f, -0.13460640609264374f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.06680718809366226f, -0.22338956594467163f, -0.20045873522758484f, -0.1861473172903061f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 8)
				{

					w = _mm_set_ps(0.1102278083562851f, 0.209168940782547f, 0.015606212429702282f, 0.15818418562412262f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.7058255076408386f, 0.477260947227478f, -0.33622273802757263f, -0.32845598459243774f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.3500153720378876f, 0.05251981317996979f, -0.14716185629367828f, 0.413205623626709f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.3908053934574127f, -0.189931720495224f, 0.03277750685811043f, 0.36003121733665466f);
					x = _mm_load_ps1(&x2[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(0.5785036683082581f, -0.003627244383096695f, 0.1825413852930069f, -0.24736201763153076f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.1779479682445526f, -0.33477911353111267f, 0.333713561296463f, -0.43581485748291016f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.17640402913093567f, -0.3695794343948364f, -0.2897358536720276f, -0.3274582624435425f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(-0.16885808110237122f, 0.08122295886278152f, -0.6437896490097046f, 0.5102269649505615f);
					x = _mm_load_ps1(&x2[x_1][x_2][1]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.48700428009033203f, 0.18463599681854248f, -0.33594799041748047f, 0.16354332864284515f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.11682490259408951f, 0.3047132194042206f, -0.014889904297888279f, -0.09151666611433029f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.3000026345252991f, -0.08725883066654205f, -0.29930076003074646f, -0.28687483072280884f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.04932045191526413f, 0.16773930191993713f, 0.185333713889122f, 0.2739829123020172f);
					x = _mm_load_ps1(&x2[x_1][x_2][2]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.16476182639598846f, 0.16735690832138062f, 0.10225819051265717f, 0.10493841767311096f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.013003731146454811f, 0.6399206519126892f, -0.09053341299295425f, 0.026019636541604996f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.023090429604053497f, 0.10329537093639374f, -0.09484171867370605f, 0.10289964079856873f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.2012944519519806f, 0.0019858907908201218f, 0.17827294766902924f, 0.44600585103034973f);
					x = _mm_load_ps1(&x2[x_1][x_2][3]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.01659982092678547f, -0.07769566774368286f, -0.2792368233203888f, 0.30940213799476624f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.384076327085495f, -0.27024391293525696f, -0.1356266736984253f, 0.006412130780518055f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.06717890501022339f, 0.015886366367340088f, -0.37106144428253174f, -0.20696960389614105f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.10787986218929291f, 0.04142669215798378f, -0.009519574232399464f, 0.29810643196105957f);
					x = _mm_load_ps1(&x2[x_1][x_2][4]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.14384815096855164f, 0.08429709076881409f, 0.5667945146560669f, -0.04588239639997482f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.20405827462673187f, 0.27578866481781006f, 0.17192399501800537f, -0.2119988650083542f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(-0.33334967494010925f, 0.4903433322906494f, -0.4590984880924225f, -0.025904210284352303f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.4139721691608429f, 0.3775200843811035f, -0.5605248212814331f, 0.22265516221523285f);
					x = _mm_load_ps1(&x2[x_1][x_2][5]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.06729692220687866f, -0.06835262477397919f, 0.3944378197193146f, -0.10107427090406418f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.32344257831573486f, 0.5870665311813354f, 0.03967708349227905f, -0.3798269033432007f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.050046999007463455f, 0.114563949406147f, 0.3255912959575653f, 0.26234766840934753f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.2470809519290924f, 0.01021055318415165f, -0.0513596273958683f, -0.44880732893943787f);
					x = _mm_load_ps1(&x2[x_1][x_2][6]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);

					w = _mm_set_ps(-0.3369457721710205f, 0.03937719017267227f, 0.01163154561072588f, -0.021949870511889458f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.017268773168325424f, 0.5148582458496094f, 0.13206948339939117f, -0.16069889068603516f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

					w = _mm_set_ps(0.25566062331199646f, 0.3592042028903961f, 0.07077656686306f, -0.09342288970947266f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

					w = _mm_set_ps(0.34421056509017944f, -0.11115429550409317f, -0.13915129005908966f, 0.19104504585266113f);
					x = _mm_load_ps1(&x2[x_1][x_2][7]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][12]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x3[x_out_1][x_out_2][12], x);
				}
			}
		}
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
		}
	}
	static float x4[4][4][16] = {0};
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
		}
	}
	static float x5 alignas(16) [4][4][32] = {0};
	static __m128i cx5 alignas(16) [4][4][32];
	static unsigned char cx_in5 alignas(16) [4][4][16];

	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 4; j++)
	        for (int k = 0; k < 16; k++)
	            cx_in5[i][j][k] = x4[i][j][k] / 0.016357818618416786f;

	for (int i = 0; i < 4; i += 1)
	{
		for (int j = 0; j < 4; j += 1)
		{
			x5[i][j][0] = -0.19863174855709076f;
			cx5[i][j][0] = _mm_setzero_si128();
			x5[i][j][1] = 0.09945124387741089f;
			cx5[i][j][1] = _mm_setzero_si128();
			x5[i][j][2] = -0.3467791974544525f;
			cx5[i][j][2] = _mm_setzero_si128();
			x5[i][j][3] = -0.026343027129769325f;
			cx5[i][j][3] = _mm_setzero_si128();
			x5[i][j][4] = -0.03656141832470894f;
			cx5[i][j][4] = _mm_setzero_si128();
			x5[i][j][5] = 0.06388148665428162f;
			cx5[i][j][5] = _mm_setzero_si128();
			x5[i][j][6] = -0.09744719415903091f;
			cx5[i][j][6] = _mm_setzero_si128();
			x5[i][j][7] = 0.02247576415538788f;
			cx5[i][j][7] = _mm_setzero_si128();
			x5[i][j][8] = -0.3631933629512787f;
			cx5[i][j][8] = _mm_setzero_si128();
			x5[i][j][9] = -0.01470577996224165f;
			cx5[i][j][9] = _mm_setzero_si128();
			x5[i][j][10] = -0.1393696814775467f;
			cx5[i][j][10] = _mm_setzero_si128();
			x5[i][j][11] = -0.033839378505945206f;
			cx5[i][j][11] = _mm_setzero_si128();
			x5[i][j][12] = -0.10633030533790588f;
			cx5[i][j][12] = _mm_setzero_si128();
			x5[i][j][13] = 0.051368821412324905f;
			cx5[i][j][13] = _mm_setzero_si128();
			x5[i][j][14] = -0.04893682152032852f;
			cx5[i][j][14] = _mm_setzero_si128();
			x5[i][j][15] = -0.1550164669752121f;
			cx5[i][j][15] = _mm_setzero_si128();
			x5[i][j][16] = -0.02287936955690384f;
			cx5[i][j][16] = _mm_setzero_si128();
			x5[i][j][17] = -0.603276252746582f;
			cx5[i][j][17] = _mm_setzero_si128();
			x5[i][j][18] = -0.05335238575935364f;
			cx5[i][j][18] = _mm_setzero_si128();
			x5[i][j][19] = -0.19695788621902466f;
			cx5[i][j][19] = _mm_setzero_si128();
			x5[i][j][20] = 0.1581440269947052f;
			cx5[i][j][20] = _mm_setzero_si128();
			x5[i][j][21] = 0.10608666390180588f;
			cx5[i][j][21] = _mm_setzero_si128();
			x5[i][j][22] = -0.05032450333237648f;
			cx5[i][j][22] = _mm_setzero_si128();
			x5[i][j][23] = -0.1295873075723648f;
			cx5[i][j][23] = _mm_setzero_si128();
			x5[i][j][24] = -0.13460379838943481f;
			cx5[i][j][24] = _mm_setzero_si128();
			x5[i][j][25] = 0.14332135021686554f;
			cx5[i][j][25] = _mm_setzero_si128();
			x5[i][j][26] = 0.09517046809196472f;
			cx5[i][j][26] = _mm_setzero_si128();
			x5[i][j][27] = -0.05684333294630051f;
			cx5[i][j][27] = _mm_setzero_si128();
			x5[i][j][28] = -0.0072254096157848835f;
			cx5[i][j][28] = _mm_setzero_si128();
			x5[i][j][29] = 0.08173099160194397f;
			cx5[i][j][29] = _mm_setzero_si128();
			x5[i][j][30] = 0.10357213020324707f;
			cx5[i][j][30] = _mm_setzero_si128();
			x5[i][j][31] = 0.04398728162050247f;
			cx5[i][j][31] = _mm_setzero_si128();
		}
	}
	for (int ix = -1; ix < 3; ix += 1)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 1) / 1;
		for (int jx = -1; jx < 3; jx += 1)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 1) / 1;
			x_1 = ix + 0;
			if (x_1 >= 0 && x_1 < 4)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -20, -4, -31, 15, 43, 28, -2, -11, -28, 38, 1, -1, 22, 25, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-50, 0, -35, 31, 19, 1, -31, -14, -1, 14, 7, -35, -25, 20, -26, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, 3, 24, 1, -1, -2, 27, 30, 14, 18, 16, 25, -18, -17, 5, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(21, 15, 4, 2, 9, -15, -24, 0, -6, 28, 10, -19, 18, -18, 1, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 27, -35, 29, 11, 3, 1, 22, 15, 8, 6, -33, 8, 2, 0, 23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -42, -4, -22, -29, 21, -31, 42, 20, -2, 9, -2, -8, -14, -22, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 5, -2, 10, -2, -8, 4, -13, 3, 2, -12, 27, -2, -6, 3, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, -25, 4, 20, 3, 1, -27, 26, -5, -50, 63, -42, -25, 0, -19, 29);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 22, 11, 8, 3, -40, -26, 22, 15, 10, 10, 18, -22, -3, -15, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, 3, 10, -8, 26, 31, -10, -2, -3, -12, -46, -11, 19, 9, -1, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(10, 41, -4, 11, -15, -11, -42, -19, -3, 9, 1, -3, 7, -8, 0, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, 19, -12, 30, 16, 2, 25, 12, -8, 63, -27, -7, 9, 7, 7, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, 33, -49, -2, 17, 5, 38, 0, 1, -20, -113, -21, 1, 3, -28, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-41, 39, -25, 0, 1, 16, 32, -25, 2, -19, -5, -19, 16, -7, 11, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(16, -12, -8, -33, -10, 21, -49, 0, 25, -17, 26, 17, -1, -13, 1, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 26, -16, 1, 0, -23, -27, -15, -7, 30, -11, -7, 14, -14, -3, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, -8, 0, 0, -6, -11, -10, 1, 7, -2, -4, -11, 7, 6, 11, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, 8, 16, 29, 10, -8, -13, 12, 32, 25, -10, 24, 10, -26, -45, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, 7, -31, 7, -4, -14, 37, 18, 0, -24, -17, 18, -7, -8, 4, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, 41, -26, -4, 18, -62, -58, -7, -54, 9, -7, -4, 5, -1, 13, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -5, -2, 12, 2, 13, -32, -56, 0, 7, 33, -52, 4, 1, 16, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(30, 15, -6, 14, 24, -8, -1, 26, -3, -7, 1, -4, 15, 5, -5, 14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(14, 23, -20, 5, 9, -18, 15, 14, 46, -7, 17, -15, -12, -17, 2, 24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(13, 10, -46, 12, -14, -9, -25, -38, -26, 12, -35, 4, 2, -7, 19, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-32, -15, 0, -14, 12, 11, -3, -2, -14, -1, -27, 7, 5, 22, 9, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, 13, -24, -21, 16, 11, -13, -16, -1, 11, -16, 0, 19, 26, 24, -35);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-32, -1, 9, -13, 23, 20, -9, -21, -53, 18, -53, -10, 17, 0, 47, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-30, 41, -48, 2, -1, 29, 14, 12, 28, 30, 0, 49, 25, 15, 20, 26);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-41, 18, 0, 1, 17, -6, 1, -2, 23, 0, -2, -9, 15, -7, 30, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-55, 7, -32, 11, 10, 10, 11, 11, -6, 55, -35, 16, 17, -2, 29, 11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-77, 3, -9, -11, 14, 2, 25, -16, 18, 34, -12, -13, 27, -10, 22, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-37, 13, 6, 33, 23, 16, 6, -5, -26, -2, 18, -4, 14, 26, -28, 26);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-44, -20, -62, -40, 21, 27, -3, -21, 49, -26, -47, -53, 0, 3, 46, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, 17, -3, 18, 20, 12, -19, -47, 6, 10, -30, -34, -23, -8, -16, 16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(28, -21, 0, -13, -26, 14, 17, 24, -7, 4, 4, -1, 0, -30, -44, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(12, 5, 2, -18, -9, -9, 13, 13, -13, -16, 31, 7, -1, -12, -7, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-33, -4, -37, 0, 7, -14, -1, -27, -15, 36, -57, 23, -7, -5, 5, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(18, -9, 24, -12, -34, 9, 1, 59, 33, 16, 0, -5, 2, -48, 23, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(48, 9, -30, 11, 0, -43, 27, 25, -25, -11, 12, -18, 31, -16, 13, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-46, -25, 8, -9, 6, -27, 10, -1, 13, 0, 40, -23, -6, 10, -88, 20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, 3, 8, 5, -27, -7, 3, 10, 27, -6, 10, 35, -49, -22, -13, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(13, -3, -8, 1, 14, 9, 21, -16, -55, -27, -50, 11, -8, -30, -8, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, 12, 7, 11, -18, -2, 27, 3, -18, 5, 25, 2, 15, -18, 0, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, 7, -22, 8, 2, 2, 9, -10, 0, -17, -55, 2, 5, -6, 22, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -10, -52, -33, 3, -6, -4, -19, -22, 10, 3, -52, 31, 9, -117, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-42, 6, -33, -27, -9, 7, 42, 26, 13, -47, 21, -33, 18, 0, -38, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(70, -30, 1, -16, -38, 37, 3, 17, 49, -5, 39, 21, 1, -9, 12, -37);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 2, 7, -5, -8, -8, 7, 6, 1, -21, 11, 14, -9, -12, 12, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 3, -6, 0, 0, -11, -10, 0, 0, 6, -1, -5, -8, -11, 7, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 3, 9, 19, 0, -5, -12, 20, 30, -9, -48, 1, 0, 9, 16, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -12, -66, -3, 0, -50, 5, -38, -51, 1, 3, -38, 43, -15, -62, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(3, 11, 4, 15, 0, -12, -26, -23, -1, 18, 3, 22, -1, -7, -3, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-30, 29, -44, -53, -7, -25, -21, -53, -17, -41, -96, -26, -20, 12, 31, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-30, 3, -8, 18, -3, 1, 35, -33, -9, 8, -41, 10, -6, -1, -18, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(17, 11, -9, -38, 1, 7, -25, -4, 9, 14, -28, 28, -14, -14, 17, 18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(22, 18, -9, 0, -14, -12, 27, -41, -33, 9, 18, -8, 40, -1, 7, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, 23, -59, 5, -18, -6, -33, -33, -17, 2, -43, 4, -13, 6, 8, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-28, 0, -2, 6, 5, 20, 40, -24, -9, 7, -12, 10, -28, -9, 2, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 3, 1, -10, 16, -12, -3, -95, -12, -26, -75, 3, -18, -13, -1, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, 6, -17, 15, -2, -4, -24, -3, 9, 28, -29, 21, -25, -20, -25, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, 22, -29, 5, -10, 3, -3, -40, -17, 21, 38, -7, 6, -7, -8, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-44, -19, -20, 14, 9, -43, 0, -58, 2, 2, -11, -5, -1, -3, -7, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-45, -11, -28, -18, 4, -34, -10, -73, 31, -11, -24, -27, -4, 7, -29, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-60, -18, -2, 13, -7, -7, 5, 6, -17, 5, 23, -7, -11, 8, -45, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-40, -1, -73, -50, 15, 5, -17, -4, -16, -26, 8, -42, 8, 20, -4, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -22, -21, -51, 19, -22, -64, -46, -84, -29, -47, 23, 8, 19, 11, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-35, -46, -11, 14, 7, 0, -31, -1, 2, -38, 0, 0, 23, -10, -2, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-38, -12, 2, 7, -15, 17, 16, 3, 7, 3, -52, -6, 25, -9, -19, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, 21, -15, -2, 25, 17, 12, -13, 4, 11, -16, 16, 23, 2, 13, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-57, -6, -3, -1, 4, -21, 26, 28, 29, -17, 23, 2, 3, -52, -12, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(30, 1, -23, -17, -6, -13, 28, 6, -5, -7, 19, -5, 21, -13, -15, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 18, 11, -10, 26, -8, -11, 34, -8, 8, -1, -8, 24, 7, -11, 21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(26, 11, 18, 20, -8, -3, 1, 18, -5, -4, -31, 13, -15, -28, -13, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(36, 23, 2, -5, 27, 23, -21, -2, -4, 13, -22, 36, 23, -9, -7, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, -2, -27, -5, -19, -7, 24, -3, -14, -43, 3, -29, 51, -34, -64, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 15, 20, 19, 0, 40, 5, -7, 0, 0, -14, 7, -2, -24, 22, -3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-32, -8, -7, -12, 6, -1, -10, -28, -31, -6, -22, -86, -14, 0, -39, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, -1, -22, -44, 20, -35, 9, 2, -24, -23, 32, -19, 0, 19, -25, 35);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-51, -22, 5, -17, -25, -4, 32, 7, 25, -20, 18, 1, -12, -34, 7, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-37, -1, -13, -9, -4, -9, 39, -8, -5, -9, 6, -10, 37, -36, 4, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -3, -8, -1, 3, -10, -1, 4, 4, -1, 8, -6, 2, -11, -1, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 11, 7, 14, 23, -17, 12, 21, -17, -8, -7, 35, 8, -15, 24, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, 3, -67, -13, 0, -16, -26, -46, -98, -20, -18, -35, 33, -2, -34, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 9, -4, 7, -11, -12, 39, 15, -8, 5, 0, 1, 14, -28, 11, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(20, 25, 3, 11, 16, -3, 5, 2, 34, 7, -36, 40, 20, -7, 7, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-76, 14, 7, -7, 29, -11, 11, -11, -10, 26, 3, 9, 2, 12, 4, 25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(13, -12, -20, -24, 27, -8, 13, 16, 34, 18, 27, -37, 40, -4, 17, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-95, -7, -31, -4, -7, -28, 33, -10, -19, 0, 15, -25, 57, -33, -40, 28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, 35, 13, 1, -14, -36, 12, -14, 23, -3, -11, 14, 18, -21, -7, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -3, 21, 2, 4, 21, 18, -26, -20, -44, 15, 13, -20, 0, -17, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(45, -12, 35, -23, 8, -4, -4, -27, -19, -18, 24, -12, -8, 0, -8, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, 6, -24, 7, -17, -12, 12, 17, 33, 4, -13, -38, -54, 10, -20, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-54, 13, -1, -9, -10, 6, -6, 4, -17, -13, -29, 11, 22, -12, -45, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-41, -4, 0, 42, 0, -5, 11, -66, -2, -26, -14, -23, -6, 17, 16, 28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, 9, 22, 39, -8, 3, -20, -73, 27, -33, -17, 13, -12, 6, -25, 38);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, 22, 15, -20, 15, -7, 0, -22, -29, -14, -15, -29, 15, -6, -5, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
			}
			x_1 = ix + 1;
			if (x_1 >= 0 && x_1 < 4)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-20, 12, -29, 15, -10, 0, -8, -41, -10, 20, -7, 31, -25, -8, 12, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-28, -14, -4, 8, -5, -32, -8, -6, 6, 1, 0, -24, 19, 2, -22, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(10, -37, 7, -56, 1, -8, 17, 15, 10, 20, 38, 35, -19, -1, 9, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, -32, 17, -19, 0, -9, -12, 1, 18, -17, 17, 21, -15, -17, -13, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -6, 7, 1, 8, -12, 0, -5, -35, 6, -25, -54, -6, 21, -20, 30);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -21, -5, -20, -7, 33, -7, 19, -19, -13, 27, 20, 0, 10, 2, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(5, -2, 4, -4, -12, 1, 8, -10, 44, 16, 14, 23, -34, -9, -11, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-49, -25, -9, 14, 5, -13, 20, -24, -8, 5, 12, -9, -12, 16, -56, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 6, 0, -13, 18, -2, -7, -11, -18, 28, -6, 12, -24, 1, -34, -23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(24, 8, -10, 36, 12, -2, -21, 5, 0, -39, 10, -62, 12, 4, -29, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(37, -15, -2, -4, -16, 13, -22, 47, -2, 12, 19, 12, -2, -5, -12, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(16, 19, -16, -6, -8, 4, -9, 10, -16, -35, -46, 18, 7, 4, 0, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-30, -21, 10, 2, -9, 34, 28, 58, 5, 14, 9, 16, -11, -15, 24, 18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-51, -18, -5, 11, -4, -5, 34, 29, -49, -5, 22, -15, 30, -9, -6, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-42, -39, 14, -11, 21, 1, 2, 8, -38, -20, 13, 2, -3, 2, -29, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(48, -34, 17, -18, -25, 33, -4, 0, 7, 2, 4, 12, -13, 0, -21, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 11, 2, 6, -12, -4, -8, -4, -7, -8, 3, 1, -7, 0, -5, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(39, 20, 10, 24, -28, 0, -3, -36, 32, 30, 15, 28, -68, -18, 15, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(19, 7, 23, -12, -18, -9, 17, 46, 2, 3, 23, 10, -5, -26, 29, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(31, -24, -8, -5, 1, 46, -32, -2, -15, 42, 3, -57, -10, 13, -21, -34);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, 1, -22, 19, 0, -27, 21, -62, 14, -21, -124, -30, -31, -4, 20, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, 6, -12, 26, 6, 0, -24, 19, -52, -43, -5, -45, -17, 16, -32, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(34, -20, -3, -6, 9, -27, 1, -26, 42, -22, -5, 0, 7, -15, -36, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(20, 25, 18, -2, -15, -12, -26, 23, -18, 41, 22, 6, -6, -14, -29, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(40, 0, -46, 4, 11, 20, -45, -19, -16, -6, -35, -64, 7, 16, -8, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(14, 12, 0, 19, 9, 25, 0, 27, -31, -78, -36, 11, 2, -6, 28, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-41, -22, -19, 8, 18, 27, -13, 8, -34, 1, -83, -10, -4, -22, 9, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-43, 21, -24, 6, 3, 0, 21, -3, -8, 0, 4, -68, 10, 14, 25, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(30, -23, 14, -30, -2, 7, 18, -16, 2, -7, 30, 0, -21, -3, -89, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-127, -14, -24, 24, -6, 8, 11, -18, 7, 7, -9, -7, 17, -11, 12, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-67, 3, -9, 18, 16, -26, 16, -59, 30, 1, -17, -50, -34, 7, 37, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -18, -1, 23, -9, 0, 14, 21, -38, -15, -5, -42, 1, -17, -49, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(27, 10, -79, 27, -2, -20, -23, -22, -5, 7, 11, -22, 3, -10, 0, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 0, 5, 5, -8, -17, -8, -27, -29, -13, -35, -28, -16, 8, -2, 16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(22, -34, -14, -7, -15, 0, 0, 55, 16, -20, -7, 42, -23, -2, 2, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(75, -19, -14, -2, -33, 25, -26, 42, 19, 0, -4, 10, -17, -7, 17, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-4, 0, -13, -2, -19, -1, -5, -13, -25, -9, -55, 11, -2, 18, -4, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, -40, 4, -24, -19, 16, 19, -3, 25, 0, 64, -11, -20, -17, -4, -23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(47, -1, 24, -20, -28, 9, -30, 27, 9, -35, 9, 4, -24, -25, -10, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-23, -4, -7, -24, 11, -19, -32, -6, -3, -21, 10, 15, -31, -1, -33, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -13, 15, 12, -20, 15, 12, 20, 15, 0, 3, 30, -52, -27, 14, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(19, -3, -18, -5, 15, -6, 5, -16, -39, -8, -108, 30, -30, -6, 35, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(24, -17, 23, -9, -26, -3, 17, 26, 26, -19, 13, 26, -31, -45, 9, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(4, 0, -17, -18, -1, -3, -20, -31, -26, -45, -85, -10, -30, -2, -2, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(10, -8, 5, 6, -15, -25, -15, 5, -7, -33, -15, 2, 12, -3, -4, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-28, 4, 0, 9, -7, -35, -7, 2, 1, -32, -2, 1, -1, -2, -19, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, -21, -2, -23, 18, -13, 11, 14, 46, 9, 17, -11, -33, 6, 0, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(45, -19, 15, -37, -36, 32, 5, 30, 42, -8, 2, 28, -62, -18, 1, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -12, 4, -10, -10, -6, 4, 0, 8, 8, 1, -3, -8, -13, -7, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -14, 6, 51, 2, 6, -25, 6, 50, 18, -47, 27, -29, -11, -2, -28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, 5, 2, -23, -5, 15, -7, 0, -10, -36, -6, 30, -8, -23, -7, 9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -18, 5, -11, -26, -8, 12, 20, 43, -27, 21, 22, -27, -16, -1, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(10, 21, -16, 35, 2, 19, -13, -81, 12, 3, -104, -46, -30, -19, -28, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-9, 3, -14, 5, 0, 9, -16, -49, -71, -20, -35, -7, -11, 7, -8, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -7, -20, -25, -12, -21, -17, -35, 1, -15, -38, 16, -26, -14, -15, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(19, -22, 35, 3, -22, -6, 21, 20, -1, -23, 5, 7, -11, -47, 2, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 4, 5, -8, 5, -24, -15, -4, 9, 12, 40, 6, -53, -16, -16, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, 2, -4, 23, -14, 4, -8, -29, -81, -60, -52, -1, -31, -11, -15, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, -9, -24, -3, -6, 12, -23, -39, -2, -24, -124, -31, -23, -26, -5, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(21, -27, -16, 1, -18, -33, 10, -40, -10, 20, -24, -48, -6, -4, -17, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(58, -14, 2, -14, -14, 11, -26, 0, -35, 7, -23, -41, -1, 0, 21, -12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-94, -13, -29, 2, -3, -14, -9, -33, -10, -22, -20, -37, -8, -35, -19, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-26, -7, 8, 9, 6, -52, -35, -58, -3, -20, -33, -44, -11, -19, -5, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-65, -15, -13, -1, -18, -27, 14, 10, -39, -46, 2, -44, -24, 9, -29, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-54, 20, -47, -4, 5, -23, 8, 6, -17, -18, -29, -46, 39, -15, -33, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(5, 12, -6, 20, 37, 22, -35, -28, -17, 20, -47, 8, 13, 24, 13, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-33, -17, -18, 1, 14, 18, -7, 10, 22, -28, -23, 42, 17, -3, 0, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-13, -6, 20, -6, -13, -23, 39, 41, 54, -32, 9, 14, -11, -38, -5, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-44, 0, -13, -3, -6, 19, 4, 1, -19, -10, -21, -25, -8, 4, 10, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(57, -22, -28, -5, -15, 57, -50, 2, 29, -11, -2, -19, 0, -15, 28, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(53, -56, -4, 11, -8, 44, -24, 20, 68, -10, 1, 0, -21, -32, -1, -26);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -3, -15, -2, 26, -7, -18, -24, -12, 28, 12, 3, -19, 1, -15, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(31, -3, 18, -13, -14, 18, 31, 21, 29, -21, 5, 29, -12, -12, -13, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -5, -24, 12, -6, 31, -39, -25, -52, -10, -71, 27, -7, 6, 16, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(31, 0, -1, -12, -23, 17, 46, 14, 10, -26, 12, 23, 23, -23, 12, 4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-31, 8, 1, -10, 6, 3, -17, 6, -48, -47, -40, 17, -1, 28, -11, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(18, 21, -31, -13, 7, 6, 17, -29, 23, -31, -37, -29, -3, 9, 12, 28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(43, 6, 20, 0, 18, -36, 5, 21, 3, -40, 6, 25, 16, 27, -2, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-3, -15, -7, -50, 12, 31, 16, 21, -10, -11, 40, -17, -7, 7, 23, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(43, -25, 3, -11, -17, 22, 32, 49, 44, -15, -3, 5, 25, -21, 11, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-5, 6, 0, -12, -7, -1, 1, -4, -7, 0, -11, -1, -10, 9, -12, 7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-6, -7, 2, -3, -4, -9, -37, -14, 13, 22, 31, 33, -24, -10, -35, -22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-66, 1, -21, 5, 5, -82, -32, -25, -17, -9, -2, -10, 37, -13, -4, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -10, 0, -32, -16, 28, 20, 21, 13, -19, 21, 6, -9, -15, 14, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(30, -7, 41, -37, -18, 14, 2, -23, 18, -3, 7, -4, 8, -9, -7, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-31, 15, -19, -40, 7, -7, 1, -23, -70, -34, -13, -1, -4, 8, 2, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, 32, 15, -8, -2, 0, -11, 3, 32, 14, 28, 5, 12, -23, -23, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-14, -29, -7, 6, -45, 14, 42, 14, 5, -24, 1, 22, 30, -61, -7, 20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(24, -3, 14, 10, -18, 18, 19, -4, 41, 20, 15, 32, -22, -23, 16, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(41, 1, 26, -16, 13, -39, -23, 8, -102, -5, 24, 19, 21, 0, -25, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(24, -8, 17, -11, 15, 8, -31, -15, -24, -25, -59, 32, 19, 3, -1, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 6, -4, 20, -13, -27, 40, -53, 15, 14, -42, -2, -11, 26, -7, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -3, 31, 9, -32, 26, 47, 5, 44, 0, 22, 26, -10, -57, -1, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-77, -17, -11, 11, 0, -23, -19, -15, 13, -26, 8, -32, 4, -6, -30, 29);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 7, 25, -22, 2, 6, -18, -52, -3, 13, 9, -18, -1, 35, 15, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-51, 7, -18, -29, 19, 1, 0, 3, -5, 6, 4, -3, -14, 8, 6, 28);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
			}
			x_1 = ix + 2;
			if (x_1 >= 0 && x_1 < 4)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(38, 30, 16, -12, -26, 10, -33, 18, -37, 7, 22, -8, -9, -29, -49, 21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-40, -15, -8, -56, 0, -50, 8, -18, 20, 27, -2, -50, 46, -18, -18, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -22, 15, -52, 15, 45, 24, 0, 8, 1, 19, -15, -8, 22, 11, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -55, -7, -87, 1, 63, 13, -11, -39, -4, 33, 21, 2, 29, -24, -27);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-12, 12, 22, -17, 6, 19, -3, 33, -37, -37, 5, -25, 18, -3, -37, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-25, -10, 14, -27, 12, -23, 13, 8, -30, 13, 0, -28, -3, 7, 13, 13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -25, 12, -32, 5, 14, -10, 18, 16, 20, 13, -3, 14, -3, 26, -23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, 17, 20, 0, 12, -4, 35, -23, -8, 15, -48, -21, 16, 22, -17, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(4, -21, -5, -36, 18, 20, 21, 1, -59, 31, 27, 35, 26, 6, -61, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(19, -18, -10, 9, 14, -12, -7, 2, -38, 22, -2, -65, -2, -2, -42, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -50, -3, -30, 10, 30, -20, 11, -15, 6, 16, 13, 0, 11, 0, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-38, -15, 2, -4, 3, 25, 4, 0, -17, -11, -28, -25, -8, -2, 4, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(13, -34, 7, -40, -25, 7, -14, 48, 52, 37, 29, -6, 12, 22, 10, -47);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(37, -28, -1, -28, -27, -4, -22, 16, 43, 1, 18, 3, -8, 0, 27, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-17, 13, 12, -18, 6, -1, -19, -6, 14, 18, 61, -23, -46, 4, -28, 33);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(22, -66, -6, -53, 23, 19, 5, 24, -12, -15, 25, 4, 5, -3, 5, -11);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(2, 9, -10, -6, -5, -7, -8, 8, 2, -11, -6, -12, -6, -10, 3, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(42, -1, -60, -3, -5, 32, -12, -37, 22, 8, 21, -6, -10, -62, 0, 10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, -35, -6, -42, -12, 30, 2, 21, 17, 19, 41, 2, 0, 8, 25, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(4, -20, -19, -25, 17, 9, -33, 16, 3, 21, 18, -22, -2, 23, -11, -56);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-37, 6, -2, -1, 33, -6, 12, -14, 49, -24, -34, -7, -6, 28, 8, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(23, -12, 32, -16, -3, -12, -1, 37, -36, -28, 2, -32, 3, 7, -70, -6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 11, 8, -39, 21, -19, 20, -35, -15, -5, -49, -12, -22, 0, -24, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(18, -47, -7, -21, -21, 15, -28, -11, -2, 4, 29, 33, -3, 22, -4, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-35, -12, 15, -14, 39, 11, -36, 6, -45, -22, -17, 17, 0, 16, -40, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, -17, 7, 25, -14, 28, 11, 0, -59, -9, -8, -29, -15, -34, -21, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-10, 18, 4, 23, 18, -10, 0, -2, 51, 34, 9, -34, 20, 0, -5, 12);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(40, -11, 26, -23, 9, -27, -19, -1, 29, 6, -21, -18, 4, -7, -26, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(59, -46, -7, -29, 0, 33, -21, 3, 32, -2, 6, -14, 19, -11, -36, -31);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-97, 5, 27, -41, 7, -25, 9, -12, 40, 33, -25, -17, 31, 20, 0, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-52, 9, 19, -71, 4, -53, -18, -22, 12, 17, -68, -25, 4, 6, 1, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-21, -14, 20, -6, 4, 23, -2, 19, -15, 0, 19, -31, 23, 1, -14, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(20, -4, 10, 10, -15, -2, 25, 14, -22, -13, 45, 23, 10, -44, -1, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(41, 36, 10, -22, -19, -14, 25, -2, 53, 30, 14, 0, 10, 0, -4, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(12, 12, 1, -56, 1, -8, -17, 28, 5, -2, -14, -7, 9, -1, 14, -47);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, -26, -8, -53, 18, -18, -2, 24, 27, 23, 26, -22, 2, 18, 22, -31);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, 6, 1, 6, 3, -39, 4, 4, -23, -31, 18, -4, -3, 9, -72, 21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-24, -15, -29, -45, -16, 19, -4, -43, -5, 16, 30, -5, 1, 9, -7, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(38, 2, -38, -43, -2, 57, -11, -2, -23, 4, 5, 7, 8, 11, -3, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-45, -16, -23, -31, -3, 20, 3, -47, 24, -15, -57, 9, 1, 10, 10, -13);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-18, -31, 6, -56, 6, 4, 9, 18, -23, 47, 37, 11, -19, -12, 10, -8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-36, -1, 1, -12, 17, 18, 15, -17, -38, -8, -108, 37, -2, -7, -3, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-1, -46, 8, -45, -7, 26, 4, 20, 49, 17, 54, 4, -34, -6, 12, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-17, 0, 8, -13, 7, 25, -10, 14, -86, -22, -68, -17, 10, -17, -21, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(52, -27, -30, -16, -23, -5, -41, 19, 2, 9, -40, -17, 8, 0, 6, -26);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(36, -33, -7, -13, -30, 20, 15, 37, 14, -13, 41, 8, -25, -37, -4, -18);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-44, -8, 10, -65, 0, -27, 16, -6, -8, 33, -57, -37, -36, 21, -8, 0);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(16, -24, -5, -49, 3, 23, 32, -2, 25, 14, 63, 8, -27, 0, 10, -31);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, 5, 3, -14, -13, -5, 0, -7, 7, -12, 0, 8, -2, 5, -4, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(8, -4, -32, 0, -2, 34, -13, -11, 13, 24, -28, -30, -27, -34, -50, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(36, -19, -21, -15, -12, 21, -14, 44, 13, -57, -26, 3, 10, -4, 12, -20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -29, 16, -33, 0, 3, -11, 22, 32, -7, 37, 26, -29, -8, -1, -24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -20, 12, -15, -1, 4, -5, -7, -4, -6, -60, 9, -2, 5, -7, -21);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-11, -4, 4, 23, 12, -8, 15, 16, 0, -19, 6, 22, -11, 6, -63, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -5, -4, 17, 5, -16, 7, -44, -9, -19, -5, 26, -8, 17, -34, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(27, -39, 13, -21, -20, 17, -34, 11, 44, -3, 44, 31, -36, -5, 24, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-16, -53, -22, -25, 9, 14, -20, -5, 0, 22, 18, -5, -21, 11, -37, -32);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -3, -1, -25, 18, 8, -3, -15, -66, 40, -39, -6, -32, -8, 4, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-19, -28, -12, -24, 0, -1, 3, -9, 8, 26, -58, -58, -34, -11, 7, 5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(9, -3, 36, -24, -15, -28, -4, 10, 22, -15, 0, 11, 23, -14, -53, -15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(7, -56, 6, -25, -3, 0, -16, 49, 20, -4, 33, -27, -17, 6, 20, 2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-82, -18, 19, -24, -1, -53, 20, 5, 27, 4, -32, -31, -7, 17, -3, -14);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-29, -4, 4, -59, -26, -22, 15, -29, -14, -28, -75, -8, -4, 5, -25, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-8, -21, -2, -4, 10, 12, 15, 33, 15, -32, -8, 9, -2, 7, -71, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 4)
				{

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(20, -1, 20, 2, -13, 2, 9, 25, 7, -6, -18, 19, 14, -52, -22, 15);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][0]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-40, -50, -9, -35, -20, -22, -19, 40, -10, 16, 0, 17, 9, -30, -9, -40);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][1]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(17, 19, -51, -5, 36, -2, -55, -33, -3, 20, -28, -15, 27, 0, -22, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][2] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][2]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-35, 1, -15, -59, 12, 25, -9, -10, 18, 4, 28, -16, 4, 5, -36, -4);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][3] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][3]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-33, 22, -4, 1, 23, -17, 16, 8, -10, -21, 11, 6, -1, 8, -22, 24);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][4] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][4]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(18, -29, -6, 34, 21, 32, -2, -37, -81, -9, -26, 1, 20, 20, 29, 1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][5] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][5]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(0, -40, -35, -22, 14, 26, 12, 35, 17, -21, 8, 15, 12, 7, 6, -20);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][6] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][6]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-55, 15, -1, 23, 7, 40, 6, -11, 22, -19, -21, 13, 6, -9, -18, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][7] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][7]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-46, -4, -15, -30, 31, 28, 6, 17, -35, 33, 2, 22, 9, -3, 3, -39);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][8] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][8]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-36, -8, -8, -1, 13, 37, -9, -34, -13, -12, -88, 40, 12, -10, 5, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][9] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][9]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -4, -3, -19, 6, 27, 5, 8, -24, 4, 11, 8, 11, 0, 23, -10);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][10] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][10]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(6, 13, 35, 6, 8, 13, 4, 0, -52, -25, -33, 36, 16, 16, -17, 17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][11] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][11]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(21, 36, -14, 14, 1, 22, 17, -5, -56, 4, -12, -32, 8, -10, 2, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][12] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][12]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-122, -52, -33, 12, 6, -31, -62, -20, -17, -46, -26, 0, 26, 2, -7, -41);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][13] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][13]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(13, 26, 7, -8, 7, 1, -7, -4, 44, 15, -26, 0, -11, -6, -18, 22);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][14] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][14]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-32, -13, -14, -40, 26, 33, 5, 12, 18, -8, 14, -4, 33, -3, 1, -33);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][15] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][15]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-7, -1, -1, -7, 0, -12, -10, -5, -6, 2, -5, -11, 3, 0, -1, 3);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][16] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][16]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(34, -2, 4, -9, -9, -37, 12, 6, 22, 22, -1, 15, -24, -15, -16, 23);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][17] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][17]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-20, 1, 8, -20, 19, -29, 51, -23, -28, -56, 9, 20, 26, -10, -4, -16);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][18] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][18]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-2, -5, 1, 4, -1, 20, 18, 31, 5, 30, -16, 0, 19, 3, 19, -17);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][19] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][19]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-73, 0, 3, 17, -12, 22, -6, -4, -35, 37, 16, 2, 12, 16, 23, -1);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][20] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][20]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-58, -1, -4, -17, 17, 6, -8, 12, -7, 2, -5, 2, 5, 21, -8, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][21] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][21]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-36, 24, 33, 3, -30, -5, 22, -29, 44, 43, 14, 1, -20, 12, 12, 6);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][22] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][22]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(28, 1, 19, -12, -4, 17, 2, 19, 35, -21, 0, 17, 23, -30, 32, -5);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][23] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][23]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(14, -21, -2, -10, -5, 38, -14, 0, 45, 20, 51, -6, -6, 10, 24, -2);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][24] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][24]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-27, -37, 5, -38, 33, -7, -36, 15, -41, -8, -52, 43, 18, 7, 22, -25);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][25] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][25]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(1, -18, 26, -2, 0, 19, -10, 17, 19, 19, -61, 33, 6, 21, 30, -19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][26] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][26]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, 12, 1, -7, -8, -18, 17, 26, 37, 15, -24, -24, 48, -26, -9, 32);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][27] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][27]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-15, -9, -23, -25, -4, 33, 12, 7, 25, 40, 40, -17, -10, -18, 20, -9);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][28] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][28]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-61, 6, -1, 8, 0, -15, 44, -38, 58, -10, -3, 0, 6, 37, 10, 8);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][29] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][29]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-56, 38, 19, 9, 25, 13, 26, -22, 24, 3, 24, -11, 21, 19, 6, 19);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][30] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][30]);

					qx = _mm_lddqu_si128((__m128i*)&cx_in5[x_1][x_2][0]);
					qw = _mm_set_epi8(-25, 26, 4, -12, 20, 22, -1, 8, 30, -37, -11, 27, 8, 7, -16, -7);
					qx = _mm_maddubs_epi16(qx, qw);
					cx5[x_out_1][x_out_2][31] = _mm_adds_epi16(qx, cx5[x_out_1][x_out_2][31]);
				}
			}
		}
	}

	for (int i = 0; i < 4; i++)
	    for (int j = 0; j < 4; j++)
	        for (int k = 0; k < 32; k++){
	            qx = cx5[i][j][k];              
	            lo = _mm_srai_epi32(_mm_unpacklo_epi16(qx, qx), 16);
	            hi = _mm_srai_epi32(_mm_unpackhi_epi16(qx, qx), 16);
	            sum1 = _mm_hadd_epi32(hi, lo);
	            sum2 = _mm_hadd_epi32(sum1, sum1);
		        _mm_store_si128((__m128i*)res, sum2);
	            x5[i][j][k] += (res[0] + res[1]) * 0.010009966497346172f * 0.016357818618416786f;
	        }
	for (int i = 0; i < 4; i += 1)
	{
		for (int j = 0; j < 4; j += 1)
		{

			x = _mm_load_ps((float*)&x5[i][j][0]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][0], x);

			x = _mm_load_ps((float*)&x5[i][j][4]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][4], x);

			x = _mm_load_ps((float*)&x5[i][j][8]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][8], x);

			x = _mm_load_ps((float*)&x5[i][j][12]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][12], x);

			x = _mm_load_ps((float*)&x5[i][j][16]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][16], x);

			x = _mm_load_ps((float*)&x5[i][j][20]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][20], x);

			x = _mm_load_ps((float*)&x5[i][j][24]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][24], x);

			x = _mm_load_ps((float*)&x5[i][j][28]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x5[i][j][28], x);
		}
	}
	static float x6[2][2][32] = {0};
	for (int ix = 0; ix < 3; ix += 2)
	{
		int x_1, x_out_1;
		x_out_1 = ix / 2;
	for (int jx = 0; jx < 3; jx += 2)
	{
		int x_2, x_out_2;
		x_out_2 = jx / 2;
		x = _mm_load_ps((float*)&x5[ix][jx][0]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][0], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][0]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][0]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][0], x);
		x = _mm_load_ps((float*)&x5[ix][jx][4]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][4], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][4]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][4]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][4], x);
		x = _mm_load_ps((float*)&x5[ix][jx][8]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][8], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][8]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][8]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][8], x);
		x = _mm_load_ps((float*)&x5[ix][jx][12]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][12], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][12]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][12]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][12], x);
		x = _mm_load_ps((float*)&x5[ix][jx][16]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][16]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][16]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][16], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][16]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][16]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][16], x);
		x = _mm_load_ps((float*)&x5[ix][jx][20]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][20]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][20]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][20], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][20]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][20]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][20], x);
		x = _mm_load_ps((float*)&x5[ix][jx][24]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][24]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][24]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][24], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][24]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][24]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][24], x);
		x = _mm_load_ps((float*)&x5[ix][jx][28]);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 0][28]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 0][jx + 1][28]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][28], x);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 0][28]);
		x = _mm_max_ps(x, y);
		y = _mm_load_ps((float*)&x5[ix + 1][jx + 1][28]);
		x = _mm_max_ps(x, y);
		_mm_store_ps((float*)&x6[x_out_1][x_out_2][28], x);
		}
	}
	static float x7 alignas(16) [1][1][2] = {0};
	static __m128i cx7 alignas(16) [1][1][2];
	static unsigned char cx_in7 alignas(16) [2][2][32];

	for (int i = 0; i < 2; i++)
	    for (int j = 0; j < 2; j++)
	        for (int k = 0; k < 32; k++)
	            cx_in7[i][j][k] = x6[i][j][k] / 0.022600408643484116f;

	for (int i = 0; i < 1; i += 1)
	{
		for (int j = 0; j < 1; j += 1)
		{
			x7[i][j][0] = 0.8987866640090942f;
			cx7[i][j][0] = _mm_setzero_si128();
			x7[i][j][1] = -0.8987868428230286f;
			cx7[i][j][1] = _mm_setzero_si128();
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

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(-95, 5, 25, 2, 41, -97, 68, -117, 49, -22, -15, 9, -113, -48, -16, -97);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(81, -16, -44, -16, -25, 88, -63, 106, -52, 39, 10, 0, 80, 57, 23, 101);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(44, -10, 51, -106, -13, 82, 84, -56, -79, -27, 24, -55, -71, -52, 54, 0);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(-34, -14, -60, 105, 28, -81, -100, 61, 81, 23, 6, 33, 74, 59, -44, -3);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(-55, 39, -103, -91, 28, -59, 102, -54, 57, -55, 15, 25, -71, -98, 9, -75);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(67, -41, 106, 89, -22, 71, -91, 73, -78, 72, -16, -10, 40, 85, -8, 77);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(46, 127, 54, -82, 33, 59, 39, -89, -65, 55, 27, 82, -81, -68, 87, -7);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(-25, -109, -46, 56, -40, -60, -57, 78, 43, -72, -38, -78, 97, 72, -70, 0);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);
			x_1 = ix + 1;
			x_2 = jx + 0;

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(-80, -87, 43, 69, 91, -40, 45, -48, 8, -69, -43, 55, -91, -106, 94, -77);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(80, 85, -44, -83, -63, 61, -42, 23, 0, 39, 64, -51, 86, 103, -87, 79);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(34, 0, 90, -54, 68, 68, 53, -55, -70, -42, 20, -27, -37, 55, -38, -10);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(-43, 22, -61, 60, -90, -46, -69, 64, 46, 36, -46, 20, 37, -56, 54, 17);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);
			x_2 = jx + 1;

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(-58, -73, 29, -38, 37, -61, 52, -69, 46, -73, -87, 51, -44, -16, 86, -73);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][0]);
			qw = _mm_set_epi8(59, 82, -43, 25, -39, 49, -40, 73, -70, 55, 93, -57, 61, 22, -84, 72);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(47, 45, 88, -25, 57, 61, 44, -23, -87, 51, 66, 125, -60, -78, -64, -5);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][0] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][0]);

			qx = _mm_lddqu_si128((__m128i*)&cx_in7[x_1][x_2][16]);
			qw = _mm_set_epi8(-70, -82, -61, 46, -22, -43, -40, -1, 64, -25, -46, -99, 74, 70, 33, -18);
			qx = _mm_maddubs_epi16(qx, qw);
			cx7[x_out_1][x_out_2][1] = _mm_adds_epi16(qx, cx7[x_out_1][x_out_2][1]);
		}
	}

	for (int i = 0; i < 1; i++)
	    for (int j = 0; j < 1; j++)
	        for (int k = 0; k < 2; k++){
	            qx = cx7[i][j][k];              
	            lo = _mm_srai_epi32(_mm_unpacklo_epi16(qx, qx), 16);
	            hi = _mm_srai_epi32(_mm_unpackhi_epi16(qx, qx), 16);
	            sum1 = _mm_hadd_epi32(hi, lo);
	            sum2 = _mm_hadd_epi32(sum1, sum1);
		        _mm_store_si128((__m128i*)res, sum2);
	            x7[i][j][k] += (res[0] + res[1]) * 0.011132516260222188f * 0.022600408643484116f;
	        }
	static float x8[1][1][2] = {0};
	x8[0][0][0] = x7[0][0][0] / 9.642840;
	x8[0][0][1] = x7[0][0][1] / 9.642840;
	scores[0] = x8[0][0][0];
	scores[1] = x8[0][0][1];
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
		cnn(x, scores);
	 clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	printf("%f %f, %f s ", scores[0], scores[1], elapsed_secs);
#else
    cnn(x, scores);
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