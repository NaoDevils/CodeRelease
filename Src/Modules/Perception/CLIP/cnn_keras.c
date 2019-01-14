#include <emmintrin.h>
#include <math.h>
int cnn(float x0[16][16][1], int *res, float *scores)
{
__m128 w, x, y, y2, t, t2;
	for (int xi = 0; xi < 16; xi += 1)
	{
	x0[xi][0][0] = (x0[xi][0][0] - 0.565012f);
	x0[xi][1][0] = (x0[xi][1][0] - 0.565012f);
	x0[xi][2][0] = (x0[xi][2][0] - 0.565012f);
	x0[xi][3][0] = (x0[xi][3][0] - 0.565012f);
	x0[xi][4][0] = (x0[xi][4][0] - 0.565012f);
	x0[xi][5][0] = (x0[xi][5][0] - 0.565012f);
	x0[xi][6][0] = (x0[xi][6][0] - 0.565012f);
	x0[xi][7][0] = (x0[xi][7][0] - 0.565012f);
	x0[xi][8][0] = (x0[xi][8][0] - 0.565012f);
	x0[xi][9][0] = (x0[xi][9][0] - 0.565012f);
	x0[xi][10][0] = (x0[xi][10][0] - 0.565012f);
	x0[xi][11][0] = (x0[xi][11][0] - 0.565012f);
	x0[xi][12][0] = (x0[xi][12][0] - 0.565012f);
	x0[xi][13][0] = (x0[xi][13][0] - 0.565012f);
	x0[xi][14][0] = (x0[xi][14][0] - 0.565012f);
	x0[xi][15][0] = (x0[xi][15][0] - 0.565012f);
	}
	static float x1 alignas(16) [8][8][8] = {0};
	for (int i = 0; i < 8; i += 1)
	{
		for (int j = 0; j < 8; j += 1)
		{
			x1[i][j][0] = -1.1639820337295532f;
			x1[i][j][1] = -0.40259361267089844f;
			x1[i][j][2] = -0.14643387496471405f;
			x1[i][j][3] = -0.47698307037353516f;
			x1[i][j][4] = -0.1488853096961975f;
			x1[i][j][5] = -0.10877662897109985f;
			x1[i][j][6] = -0.14107418060302734f;
			x1[i][j][7] = -0.09586740285158157f;
		}
	}
	for (int ix = -1; ix < 14; ix += 2)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 1) / 2;
		for (int jx = -1; jx < 14; jx += 2)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 1) / 2;
			x_1 = ix + 0;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.36815500259399414f, 0.2382616251707077f, 0.24861818552017212f, 0.04704447090625763f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.39208894968032837f, 0.1684272289276123f, -0.2966268062591553f, -0.32302770018577576f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.09881091117858887f, -0.07198814302682877f, 0.1767655462026596f, -0.8980647921562195f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.2907719314098358f, -0.4307316541671753f, -0.11361517012119293f, 0.03393251448869705f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.060732048004865646f, -0.2065245807170868f, -0.23811431229114532f, 0.351681649684906f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.3396188020706177f, -0.16236288845539093f, -0.15469518303871155f, 0.31900545954704285f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 3;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.0944434329867363f, -0.03732433170080185f, -0.20615339279174805f, 0.08270285278558731f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.16636259853839874f, 0.030802132561802864f, -0.31548765301704407f, 0.40646275877952576f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 4;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.45815756916999817f, 0.6456849575042725f, -0.07282774150371552f, 0.4311785399913788f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.3423689901828766f, -0.42671823501586914f, -0.24099017679691315f, 0.2846662700176239f);
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

					w = _mm_set_ps(0.3723353147506714f, -0.1306941956281662f, -0.01578555628657341f, -1.0535380840301514f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.031096475198864937f, -0.23437701165676117f, -0.11534144729375839f, 0.28144100308418274f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.3613850474357605f, -0.4358682334423065f, 0.29477739334106445f, -1.4362332820892334f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.6147494316101074f, -0.2709546685218811f, 0.8396340012550354f, 0.8412612676620483f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.5385002493858337f, -0.05633149668574333f, -0.2593231499195099f, -0.03762733191251755f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.08580279350280762f, -0.14691241085529327f, 0.7520670294761658f, 0.3798695206642151f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 3;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.10542969405651093f, 0.10191262513399124f, -0.5891692638397217f, -0.5390937924385071f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.19523479044437408f, 0.3879266083240509f, 0.49727171659469604f, 0.3328346610069275f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 4;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.11325149983167648f, 1.0686166286468506f, -1.2234563827514648f, -0.6802483797073364f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.45808887481689453f, -0.578018307685852f, -0.30880990624427795f, 0.010488891042768955f);
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

					w = _mm_set_ps(-0.2557357847690582f, -0.002722930395975709f, -0.12664659321308136f, 0.4156060218811035f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.4419803321361542f, -0.13961566984653473f, -0.1973288357257843f, 0.42158421874046326f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.1406097710132599f, -0.2008584886789322f, 0.5904552936553955f, -0.5044659376144409f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.7361976504325867f, 0.20039266347885132f, 0.3281659185886383f, 0.24295112490653992f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.3813989460468292f, 0.019080106168985367f, 0.30032089352607727f, 0.016445687040686607f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.026772774755954742f, 0.28553128242492676f, 0.1689782738685608f, -0.14465534687042236f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 3;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.355643093585968f, -0.01678672805428505f, -0.13916189968585968f, -0.002979584503918886f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.15088024735450745f, 0.13853596150875092f, -0.16760234534740448f, -0.35295405983924866f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 4;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.03601348400115967f, 0.9115739464759827f, -0.30896130204200745f, 0.38438859581947327f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.5507506132125854f, 0.23279671370983124f, -0.22235959768295288f, -0.6782592535018921f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
			}
			x_1 = ix + 3;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.44040846824645996f, -0.2426244169473648f, -0.2484854906797409f, 0.19070787727832794f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.30584996938705444f, -0.1352175772190094f, -0.1606501340866089f, 0.4176222085952759f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.2773371636867523f, -0.08281869441270828f, 0.059115130454301834f, -0.2892511487007141f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.40050455927848816f, -0.2782200872898102f, -0.7150940299034119f, 0.13418297469615936f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.1219925507903099f, 0.2161741852760315f, 0.38533732295036316f, 0.19304624199867249f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.0041280388832092285f, 0.16721586883068085f, -0.44098222255706787f, -0.3970998227596283f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 3;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.332131028175354f, 0.5420764088630676f, 0.21547211706638336f, -0.01880589686334133f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.22987976670265198f, 0.004623750690370798f, 0.1041385680437088f, -0.2370479702949524f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 4;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.1499219387769699f, 0.054960183799266815f, -0.08811961859464645f, 0.0437496080994606f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.18840961158275604f, 0.12081470340490341f, 0.07147973030805588f, -0.5125039219856262f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
			}
			x_1 = ix + 4;
			if (x_1 >= 0 && x_1 < 16)
			{
				x_2 = jx + 0;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.5268585085868835f, -0.2992861866950989f, -0.20368169248104095f, 0.32816118001937866f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.18175852298736572f, -0.2576693892478943f, 0.20741961896419525f, -0.0435025691986084f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 1;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.23359639942646027f, -0.44415706396102905f, -0.11463627964258194f, -0.5303667783737183f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(-0.07642877101898193f, -0.35939547419548035f, 0.5791949033737183f, -0.36926543712615967f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 2;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(-0.17687192559242249f, -0.2994508743286133f, -0.018562788143754005f, 0.2372485250234604f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.26485323905944824f, 0.07213972508907318f, 0.0332745797932148f, -0.41248655319213867f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 3;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.05396658182144165f, -0.8008480072021484f, 0.1803804337978363f, -0.15537361800670624f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.2338552325963974f, 0.22590240836143494f, 0.01725483499467373f, -0.24144308269023895f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
				x_2 = jx + 4;
				if (x_2 >= 0 && x_2 < 16)
				{

					w = _mm_set_ps(0.60019850730896f, -0.5224833488464355f, 0.5585502982139587f, -0.05632151663303375f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][0]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][0], x);

					w = _mm_set_ps(0.46982088685035706f, 1.3504607677459717f, -0.18757867813110352f, -0.12261968106031418f);
					x = _mm_load_ps1(&x0[x_1][x_2][0]);
					y = _mm_mul_ps(w, x);
					x = _mm_load_ps((float*)&x1[x_out_1][x_out_2][4]);
					x = _mm_add_ps(x, y);
					_mm_store_ps((float*)&x1[x_out_1][x_out_2][4], x);
				}
			}
		}
	}
	for (int i = 0; i < 8; i += 1)
	{
		for (int j = 0; j < 8; j += 1)
		{

			x = _mm_load_ps((float*)&x1[i][j][0]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][0], x);

			x = _mm_load_ps((float*)&x1[i][j][4]);
			x = _mm_max_ps(x, _mm_setzero_ps());
			_mm_store_ps((float*)&x1[i][j][4], x);
		}
	}
	static float x2[4][4][8] = {0};
	for (int ix = 0; ix < 7; ix += 2)
	{
		int x_1, x_out_1;
		x_out_1 = ix / 2;
	for (int jx = 0; jx < 7; jx += 2)
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
	static float x3 alignas(16) [2][2][12] = {0};
	for (int i = 0; i < 2; i += 1)
	{
		for (int j = 0; j < 2; j += 1)
		{
			x3[i][j][0] = -0.26906102895736694f;
			x3[i][j][1] = -0.22389134764671326f;
			x3[i][j][2] = 0.5497896671295166f;
			x3[i][j][3] = 0.196365624666214f;
			x3[i][j][4] = 0.4149513244628906f;
			x3[i][j][5] = 0.018909789621829987f;
			x3[i][j][6] = 0.44122692942619324f;
			x3[i][j][7] = 0.05957222357392311f;
			x3[i][j][8] = 0.08465099334716797f;
			x3[i][j][9] = 0.1278846263885498f;
			x3[i][j][10] = -0.9384148716926575f;
			x3[i][j][11] = 0.023843631148338318f;
		}
	}
	for (int ix = -0; ix < 2; ix += 1)
	{
		int x_1, x_out_1;
		x_out_1 = (ix + 0) / 1;
		for (int jx = -0; jx < 2; jx += 1)
		{
			int x_2, x_out_2;
			x_out_2 = (jx + 0) / 1;
			x_1 = ix + 0;
			x_2 = jx + 0;

			w = _mm_set_ps(-1.025729775428772f, 0.03832109272480011f, 1.7986468076705933f, -0.05409010872244835f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.7623504400253296f, 0.34976696968078613f, -0.3053249418735504f, 0.7143646478652954f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.24565966427326202f, 1.155883550643921f, -0.9704598784446716f, -0.20309564471244812f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-1.1815494298934937f, -0.35696378350257874f, 1.4921300411224365f, -0.37866052985191345f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.7605535387992859f, -0.5324922204017639f, -0.7290998697280884f, 0.7061926126480103f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.32779595255851746f, 0.6441327333450317f, -0.036330632865428925f, 0.30073514580726624f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.23431840538978577f, -0.4372258484363556f, 0.12403952330350876f, -0.8795937895774841f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.2905047833919525f, -0.3170487582683563f, -0.620492696762085f, 0.03151652589440346f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.10342223942279816f, -0.40967443585395813f, -0.1646769940853119f, 0.18636928498744965f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.1420826017856598f, -0.5674041509628296f, 0.5675663948059082f, -1.4119242429733276f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.39149898290634155f, 0.06482311338186264f, -0.18111088871955872f, 0.3790672719478607f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.28201133012771606f, -0.4390200674533844f, -0.2749522924423218f, -0.23825037479400635f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.47493627667427063f, -0.6197095513343811f, 0.11182671785354614f, 0.3173406720161438f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.14827793836593628f, -0.03562035784125328f, 0.1415509432554245f, 0.07022528350353241f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.11240705102682114f, -0.19253677129745483f, 0.18694815039634705f, -0.3412681221961975f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.061825841665267944f, 0.7850187420845032f, 1.2425278425216675f, -0.4666546583175659f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.47412753105163574f, -0.32150933146476746f, -0.5225481390953064f, 0.26617053151130676f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.1317112147808075f, -0.10795916616916656f, 0.7928241491317749f, 0.9906252026557922f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.8117735385894775f, 0.32091274857521057f, 0.003136273007839918f, 0.016076741740107536f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.06804099678993225f, 0.6353226900100708f, 0.20313723385334015f, 0.4738140106201172f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.1416560560464859f, -0.07781573385000229f, -0.2686295509338379f, 0.7873833179473877f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.28716525435447693f, 0.458371102809906f, 0.2514767348766327f, -0.021168958395719528f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4487617611885071f, 0.12423966079950333f, 0.2694358825683594f, -0.22713975608348846f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.4274452030658722f, 0.005496435798704624f, -0.0020716702565550804f, 0.041705336421728134f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 1;

			w = _mm_set_ps(-1.9633427858352661f, 0.48311343789100647f, 0.717917799949646f, 0.3351874351501465f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.17087113857269287f, 0.21929088234901428f, -0.7184882164001465f, -0.35490691661834717f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.2863806486129761f, -0.07570140063762665f, 0.8078884482383728f, -0.15946416556835175f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.22446897625923157f, -0.5419855713844299f, 0.0765509158372879f, 0.0006440063589252532f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.4680928885936737f, -0.4950859844684601f, 0.11785932630300522f, -0.38407251238822937f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.8443167805671692f, 0.37862759828567505f, -1.009072184562683f, 0.27193209528923035f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.7876391410827637f, 0.2326885163784027f, -0.2693256139755249f, -0.5866883993148804f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.5158829689025879f, -0.19236259162425995f, 0.29864683747291565f, 0.3393835127353668f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.10313002020120621f, -0.1424189656972885f, 0.08121326565742493f, 0.6689084768295288f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.049193374812603f, 0.22688446938991547f, 0.34729713201522827f, -0.28503090143203735f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.021219884976744652f, 0.663853645324707f, 0.8144071102142334f, 0.2619195580482483f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.7576947212219238f, -0.047501321882009506f, -0.6208847761154175f, 0.06663984805345535f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.06426406651735306f, 0.0024242806248366833f, -0.4912877380847931f, -0.08694866299629211f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.16256658732891083f, 0.25578856468200684f, 0.6206519603729248f, -0.658847987651825f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.6204289197921753f, 0.32579293847084045f, 0.44916510581970215f, 0.3591967821121216f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.2800385653972626f, 0.27185386419296265f, 0.6679595708847046f, -0.06980576366186142f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.5460265874862671f, -0.42278361320495605f, 0.24193187057971954f, 0.251049667596817f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.15540866553783417f, 0.04583451524376869f, -0.20081910490989685f, 0.2726333439350128f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.6965847611427307f, -0.12178565561771393f, -0.31448546051979065f, -0.5791268944740295f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.003730841912329197f, -0.6009209752082825f, 0.04645135998725891f, -0.3068997263908386f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.3375301957130432f, -0.20283842086791992f, -0.500019371509552f, 0.4476860761642456f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.5740463137626648f, 0.03360762074589729f, 0.5515815019607544f, -0.3488120138645172f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4110354483127594f, 0.0033267412800341845f, -0.22711555659770966f, -0.44320210814476013f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.4093279242515564f, -0.1719146966934204f, 0.16593629121780396f, -0.030223237350583076f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 2;

			w = _mm_set_ps(0.05946636199951172f, 0.3192463517189026f, 0.40956223011016846f, -0.5486448407173157f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.6751942038536072f, 0.41703033447265625f, -0.08226192742586136f, -0.37431076169013977f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.260657399892807f, 1.164525032043457f, 0.7802175879478455f, 0.34499189257621765f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.9257446527481079f, 0.0820925310254097f, -0.06905051320791245f, -0.09368517994880676f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.565466046333313f, -0.9983159899711609f, 0.4104374349117279f, 0.03675060346722603f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.045834846794605255f, 0.3947482705116272f, -0.26227396726608276f, 0.8536512851715088f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.11909663677215576f, -1.2083938121795654f, 0.8891324400901794f, 0.05455053225159645f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.03437934070825577f, -0.20955026149749756f, -0.16364194452762604f, 0.4377545714378357f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.18595153093338013f, -0.02417619340121746f, 0.8559043407440186f, 0.3540111780166626f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.567298412322998f, -0.6386073231697083f, 0.29801347851753235f, 0.2677052319049835f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(1.0079245567321777f, -0.16118083894252777f, 0.4874174892902374f, 0.42823168635368347f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.4662063717842102f, 0.2424478679895401f, -1.8477466106414795f, 0.046573031693696976f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.38413235545158386f, -0.302816241979599f, -0.05506984889507294f, 0.0633084625005722f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.30694037675857544f, 0.7322812676429749f, -0.4545297622680664f, -0.8345257043838501f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.4322861433029175f, -0.36429834365844727f, 0.1224292516708374f, -0.28283244371414185f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.31380677223205566f, 0.056230396032333374f, 0.8893553018569946f, -0.0407848097383976f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.19971361756324768f, 0.19282929599285126f, 0.004818933550268412f, 0.1085144579410553f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.23731285333633423f, 0.14914001524448395f, 0.25974801182746887f, 0.637599527835846f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.9012196660041809f, -0.3260740339756012f, -1.0342531204223633f, 0.2487107217311859f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.5032209753990173f, 1.1626933813095093f, 0.41817665100097656f, 0.03643172234296799f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.48699861764907837f, -0.19656534492969513f, -0.00516918208450079f, 0.4848540425300598f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-1.479270339012146f, 0.23319441080093384f, 0.15335187315940857f, -0.058958858251571655f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.40445277094841003f, 0.31177666783332825f, 0.34027013182640076f, -0.17078107595443726f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.8434921503067017f, 0.41910260915756226f, 0.2058744877576828f, -0.12085868418216705f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_1 = ix + 1;
			x_2 = jx + 0;

			w = _mm_set_ps(1.5196925401687622f, 0.4251316487789154f, 0.2200125902891159f, -0.2990465760231018f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.21393001079559326f, -0.06670810282230377f, 0.3268226385116577f, 0.2822614908218384f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.07678229361772537f, 0.9532449245452881f, -0.35956093668937683f, 0.3224125802516937f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.6884230375289917f, -0.3476108908653259f, 1.0327084064483643f, 0.45667320489883423f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.6826978325843811f, -0.35554274916648865f, 0.9417155981063843f, 0.7705073356628418f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.4238559901714325f, -0.8659465909004211f, -0.3272429406642914f, -0.813142716884613f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.18505637347698212f, 0.13235747814178467f, 0.05166996270418167f, 0.5081993341445923f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.719070553779602f, -0.09626682847738266f, -0.20396208763122559f, 0.5780864357948303f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.49463361501693726f, 0.00989421084523201f, 0.44994136691093445f, 0.045630719512701035f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.47782501578330994f, 0.07271004468202591f, -0.04503989592194557f, -0.0685364380478859f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4453122615814209f, -0.4011401832103729f, 0.7897547483444214f, 0.35311561822891235f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.12718945741653442f, 0.18299223482608795f, 0.32476311922073364f, -0.6497527956962585f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.23132087290287018f, -0.8843704462051392f, 0.318517804145813f, -0.03825736045837402f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.41956210136413574f, 0.6097478866577148f, -0.6354959607124329f, 0.08586917817592621f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.8611118197441101f, -0.056250717490911484f, 0.12564243376255035f, 0.3177642822265625f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(1.1740500926971436f, 1.0113705396652222f, -0.037164356559515f, -0.28570112586021423f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.5146110653877258f, -0.15312835574150085f, -0.5408347249031067f, 0.004855688661336899f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.5777053833007812f, 0.43743327260017395f, 0.2348080575466156f, 0.1388060599565506f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.5138298869132996f, -0.4684254229068756f, -0.5275956392288208f, 0.7709110975265503f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4129083454608917f, -0.11550963670015335f, -0.7977661490440369f, 0.38722121715545654f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.7048358917236328f, 0.7135996222496033f, 0.32651063799858093f, 0.08217111229896545f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.39564019441604614f, -0.06153654307126999f, -0.008616983890533447f, -0.14571262896060944f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.2049553543329239f, -0.604705274105072f, 0.46671199798583984f, -0.5930740833282471f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.2843376100063324f, 0.04019276797771454f, 0.3563173711299896f, -0.16835100948810577f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 1;

			w = _mm_set_ps(0.9936837553977966f, -0.22975273430347443f, 0.6633682250976562f, -0.3687988817691803f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.18777978420257568f, 0.33962497115135193f, 1.0779359340667725f, -0.19788330793380737f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.7733761072158813f, -0.8195950984954834f, 0.16074217855930328f, 0.4648692011833191f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.2998039424419403f, 0.17328384518623352f, -0.48080211877822876f, 0.3715690076351166f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.5231637358665466f, -1.4055055379867554f, 0.7439361214637756f, 0.465999573469162f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.4093402624130249f, 0.10279867053031921f, -0.5598708987236023f, -0.12490703165531158f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.5177392959594727f, 0.24367599189281464f, 0.3486180305480957f, 0.2525833249092102f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.727705180644989f, -1.2834689617156982f, 0.060980167239904404f, 0.34451940655708313f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.5695978999137878f, 0.8537223935127258f, 0.011595181189477444f, -0.010229583829641342f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.4899085462093353f, 0.25586238503456116f, 0.3321223258972168f, -0.9865294694900513f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.3540817201137543f, 0.24805322289466858f, 0.0588429793715477f, 0.7675825953483582f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(1.1463747024536133f, -0.28371983766555786f, -0.4345301687717438f, -0.6626666784286499f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.006091822404414415f, 0.32987940311431885f, -0.13872693479061127f, 0.15665829181671143f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.5484970808029175f, -0.22373858094215393f, 0.7276843786239624f, -0.42392876744270325f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.909487783908844f, -0.39040878415107727f, 0.07770655304193497f, 0.4295085370540619f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.7152754068374634f, 0.5188273191452026f, 0.008966777473688126f, 0.46060508489608765f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4056943655014038f, -0.2036469280719757f, -1.3263481855392456f, -0.40889111161231995f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.29281502962112427f, 0.7718057036399841f, 0.4980827867984772f, -0.2788206934928894f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.3420618772506714f, -0.0646052211523056f, 0.16390284895896912f, -0.5761613249778748f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.11688035726547241f, -0.20057541131973267f, 0.1911550611257553f, 0.25552111864089966f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-1.5450611114501953f, 0.560512363910675f, 0.41201481223106384f, 0.2718960642814636f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.32678359746932983f, 0.08482702076435089f, -0.1409265100955963f, -0.08040241152048111f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.37527984380722046f, -1.1665269136428833f, 0.26951706409454346f, -0.8994285464286804f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.501697838306427f, -0.6025456786155701f, -0.13121435046195984f, 0.2876034677028656f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 2;

			w = _mm_set_ps(0.21696944534778595f, 0.01708776317536831f, -0.01807372272014618f, -0.2868138253688812f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.011816548183560371f, 0.41007113456726074f, 0.3932514786720276f, -0.3905964195728302f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.09919818490743637f, -0.24130569398403168f, 1.0237008333206177f, 0.3754555284976959f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.7575990557670593f, 0.6995046138763428f, -0.7505866885185242f, 0.44137996435165405f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.7980419993400574f, -1.154909610748291f, -1.0995090007781982f, 0.05151040852069855f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.31936115026474f, -0.6896490454673767f, -0.377041220664978f, 0.11225499212741852f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.3706240952014923f, -0.5864510536193848f, 0.21331347525119781f, 0.29747650027275085f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.42725661396980286f, 0.3629692792892456f, -0.03230629116296768f, 0.042616862803697586f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.21723461151123047f, -0.3355216085910797f, 0.06834349036216736f, 0.011548894457519054f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.32688993215560913f, -0.8945298790931702f, -0.21684493124485016f, 0.4916965067386627f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.16113799810409546f, -0.9096668362617493f, 0.9721042513847351f, 0.6455681920051575f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.6649546027183533f, 0.22846941649913788f, 0.10067669302225113f, 0.6433684825897217f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.19892780482769012f, -0.34946197271347046f, -0.12707702815532684f, 0.6983802318572998f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4663318693637848f, -0.11093293875455856f, 0.09781233966350555f, -0.39276307821273804f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.20490877330303192f, -0.42627856135368347f, -0.5103711485862732f, -0.01968708448112011f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.3019382655620575f, 0.2859356105327606f, -0.0782683789730072f, 0.6659882068634033f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.12119448930025101f, -1.9294739961624146f, -0.18073725700378418f, 0.864328920841217f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.35365429520606995f, 0.6593133211135864f, -0.34400713443756104f, 0.19367581605911255f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.2440592646598816f, 0.07053065299987793f, 0.8566873669624329f, 0.2314097285270691f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.1491335928440094f, 0.03198070451617241f, -0.4444481432437897f, -0.06890147924423218f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.03083118610084057f, -0.3439122140407562f, 0.43984872102737427f, 0.41205689311027527f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.4430864751338959f, 0.5525418519973755f, -0.33526307344436646f, -0.6447593569755554f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.10558220744132996f, -0.331899493932724f, 0.1381714791059494f, -0.7210331559181213f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.06941457092761993f, -0.1677466779947281f, -0.029985804110765457f, 0.0019840209279209375f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_1 = ix + 2;
			x_2 = jx + 0;

			w = _mm_set_ps(-0.3820731043815613f, 0.27379468083381653f, 0.23252809047698975f, 0.13428851962089539f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4634937644004822f, -0.11605807393789291f, 0.500370442867279f, -0.11867713928222656f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.44484594464302063f, 1.4266293048858643f, 0.41272979974746704f, 0.3175903856754303f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.13637953996658325f, 0.14177870750427246f, 0.791440486907959f, -0.23015378415584564f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.518997311592102f, 1.1089805364608765f, 0.5017628073692322f, 0.19293615221977234f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.8476129174232483f, -0.19305843114852905f, 0.0782834067940712f, -1.0462331771850586f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.45920059084892273f, 0.6352092027664185f, -0.05884629115462303f, -0.11580334603786469f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.07047118246555328f, 0.02604430355131626f, -0.07243786752223969f, -0.15167969465255737f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.08828109502792358f, -0.311136931180954f, 0.43293145298957825f, -0.7312607169151306f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.5665974020957947f, 0.01751953549683094f, -0.5074685215950012f, -0.18434475362300873f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.48751190304756165f, 0.421985924243927f, 0.22462321817874908f, -0.5493189692497253f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.1251465380191803f, 0.2001548558473587f, 0.15648582577705383f, -0.09409405291080475f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.3192313611507416f, -0.29221460223197937f, 0.0852595865726471f, 0.426202654838562f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.09555733948945999f, -0.26203081011772156f, -0.19436433911323547f, -0.42355483770370483f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.28537359833717346f, 0.13005118072032928f, 0.29765886068344116f, 0.20084235072135925f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.9819124341011047f, 0.6985970735549927f, -0.05681491643190384f, 0.4324880838394165f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-1.0093131065368652f, 0.14085400104522705f, -1.241331934928894f, -1.349813461303711f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.34970295429229736f, -0.014285577461123466f, 0.6429800391197205f, 0.6999763250350952f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.1026156097650528f, 0.21230214834213257f, -1.1935253143310547f, 0.9470388293266296f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-1.1616854667663574f, 0.2305692732334137f, -0.19237524271011353f, 0.19325768947601318f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.11111965775489807f, -0.047960538417100906f, 0.46765124797821045f, 0.6810798645019531f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.09598947316408157f, 0.3453850746154785f, 0.04164512827992439f, -1.0125237703323364f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.008193998597562313f, -1.563303828239441f, -0.31590917706489563f, -0.3757544159889221f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.7740792036056519f, 0.900155782699585f, 0.4169884920120239f, 0.4458288550376892f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 1;

			w = _mm_set_ps(0.08314625918865204f, 0.1743733286857605f, -0.8465866446495056f, 0.29588350653648376f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.1306663304567337f, -0.17421391606330872f, -0.5350081324577332f, 0.08876791596412659f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.6609048247337341f, -0.07406716048717499f, 0.43087634444236755f, 0.346268892288208f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.201376810669899f, 0.23309609293937683f, -0.3171862065792084f, -0.5953079462051392f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.6499323844909668f, 0.13149525225162506f, 0.237374410033226f, 0.10476560890674591f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.22491392493247986f, 0.09483255445957184f, -1.3986124992370605f, -0.19631145894527435f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.2830274701118469f, 0.4752686619758606f, -0.03771926462650299f, -0.4340820908546448f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.19721783697605133f, 0.060952845960855484f, 0.15052856504917145f, 0.26972413063049316f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.1080377846956253f, -0.027149224653840065f, -0.12086157500743866f, -0.8145626187324524f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.37767595052719116f, -0.3351616859436035f, -0.38629454374313354f, -0.7823547720909119f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.4402591288089752f, 0.43869197368621826f, -0.6575808525085449f, 0.32035550475120544f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.17207908630371094f, -0.09099286794662476f, -0.26902884244918823f, -0.25944143533706665f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.21755939722061157f, 0.33640024065971375f, 0.1304960995912552f, 0.22968296706676483f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.33615902066230774f, 0.5224842429161072f, -0.030444204807281494f, -0.48696067929267883f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.625939130783081f, 0.41858044266700745f, 0.11525388807058334f, -0.25555723905563354f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.17658138275146484f, 0.25793325901031494f, 0.7391693592071533f, -1.3627716302871704f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-1.0821079015731812f, -0.0941501036286354f, -0.8044033050537109f, 0.5738052129745483f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.6539454460144043f, 0.6559893488883972f, 0.7110530734062195f, 0.02046443149447441f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.3215272128582001f, 0.08572173863649368f, 0.540198802947998f, -0.007545514032244682f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.2748366594314575f, 0.006101035512983799f, 0.1715010553598404f, -0.20512433350086212f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.275793194770813f, 0.27417421340942383f, -0.35871464014053345f, -0.045316874980926514f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.32832762598991394f, 0.04034598916769028f, -0.7429717183113098f, -0.10954281687736511f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.21884076297283173f, -0.17092888057231903f, -0.9615089297294617f, -0.5008562803268433f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.5485455393791199f, -0.03633598983287811f, 0.014352703467011452f, -0.42262959480285645f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
			x_2 = jx + 2;

			w = _mm_set_ps(-0.9094985127449036f, 0.010724334977567196f, 0.22739726305007935f, 0.42307013273239136f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.32054510712623596f, -0.07002205401659012f, 0.750850260257721f, 0.16028550267219543f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.18684488534927368f, 0.4705534279346466f, 0.30560290813446045f, 0.46692049503326416f);
			x = _mm_load_ps1(&x2[x_1][x_2][0]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.07914426177740097f, -0.16527487337589264f, -0.5681588649749756f, -0.5315966010093689f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.04170745611190796f, -0.2753283679485321f, -0.2796439230442047f, -0.2141132950782776f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-1.371541976928711f, 0.12006919831037521f, -1.455614686012268f, -0.12168048322200775f);
			x = _mm_load_ps1(&x2[x_1][x_2][1]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.3060726523399353f, -0.27505189180374146f, -0.3562609851360321f, 0.2443210482597351f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.10521235316991806f, 0.3281453549861908f, 0.2000207006931305f, 0.42933276295661926f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.052923548966646194f, -0.41383859515190125f, -0.28661298751831055f, -0.46591854095458984f);
			x = _mm_load_ps1(&x2[x_1][x_2][2]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.16247208416461945f, -0.2774946093559265f, -0.7303014397621155f, 0.574149489402771f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.6250101327896118f, -1.6003533601760864f, 0.2691938281059265f, 0.42207518219947815f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.7148029208183289f, 0.8569856286048889f, -0.18342165648937225f, -0.18604621291160583f);
			x = _mm_load_ps1(&x2[x_1][x_2][3]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.16659197211265564f, 0.2030494064092636f, -0.26713141798973083f, 0.5601489543914795f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.1846083551645279f, -0.30714118480682373f, 0.14614714682102203f, -0.2601839601993561f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.0486433170735836f, 0.04541360214352608f, 0.05570150911808014f, -0.2974536418914795f);
			x = _mm_load_ps1(&x2[x_1][x_2][4]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(0.35084253549575806f, 0.47488656640052795f, 0.0693848505616188f, -0.2186262458562851f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(-0.05193972587585449f, -0.6178359985351562f, -0.5732073187828064f, 0.7426081299781799f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-1.2329626083374023f, 0.8631711602210999f, 0.11061392724514008f, 0.5736784338951111f);
			x = _mm_load_ps1(&x2[x_1][x_2][5]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.06491345912218094f, 0.2544211745262146f, 0.1706191599369049f, 0.675165057182312f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.09733479470014572f, -0.26894038915634155f, -0.36660003662109375f, 0.19831079244613647f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(0.5180621147155762f, 0.028713420033454895f, 0.5576784014701843f, -0.11214838922023773f);
			x = _mm_load_ps1(&x2[x_1][x_2][6]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);

			w = _mm_set_ps(-0.022689832374453545f, -0.1071406677365303f, -0.2347661405801773f, 0.07255000621080399f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][0]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][0], x);

			w = _mm_set_ps(0.39239704608917236f, 0.24480029940605164f, -0.03397008404135704f, -0.27628228068351746f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][4]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][4], x);

			w = _mm_set_ps(-0.02960893325507641f, 0.6015009880065918f, 0.005666570737957954f, 0.441517174243927f);
			x = _mm_load_ps1(&x2[x_1][x_2][7]);
			y = _mm_mul_ps(w, x);
			x = _mm_load_ps((float*)&x3[x_out_1][x_out_2][8]);
			x = _mm_add_ps(x, y);
			_mm_store_ps((float*)&x3[x_out_1][x_out_2][8], x);
		}
	}
	for (int i = 0; i < 2; i += 1)
	{
		for (int j = 0; j < 2; j += 1)
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
		}
	}
	static float x4 alignas(16) [1][1][2] = {0};
	for (int i = 0; i < 1; i += 1)
	{
		for (int j = 0; j < 1; j += 1)
		{
			x4[i][j][0] = 0.09094691276550293f;
			x4[i][j][1] = -0.09094691276550293f;
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
			x4[x_out_1][x_out_2][0] += 0.7658697366714478f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][1] += -1.0443168878555298f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][0] += -1.638003945350647f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][1] += 1.4785219430923462f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][0] += 0.5008333325386047f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][1] += -0.9578391313552856f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][0] += -0.8646127581596375f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][1] += 0.8009257912635803f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][0] += 1.2075326442718506f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][1] += -1.1493773460388184f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][0] += -1.8362102508544922f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][1] += 1.811495304107666f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][0] += 2.197758197784424f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][1] += -2.4971418380737305f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][0] += 0.5033385753631592f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][1] += -0.5333746671676636f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][0] += 1.217519760131836f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][1] += -1.289867639541626f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][0] += 0.668750524520874f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][1] += -0.4406431317329407f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][0] += -1.827593207359314f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][1] += 2.093437910079956f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][0] += -2.3459877967834473f * x3[x_1][x_2][11];
			x4[x_out_1][x_out_2][1] += 2.472778797149658f * x3[x_1][x_2][11];
			x_2 = jx + 1;
			x4[x_out_1][x_out_2][0] += 1.3993494510650635f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][1] += -1.2020072937011719f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][0] += -0.5654077529907227f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][1] += 0.6595511436462402f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][0] += 0.677900493144989f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][1] += -0.4463805854320526f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][0] += -1.1054445505142212f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][1] += 0.873143196105957f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][0] += 1.9098546504974365f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][1] += -1.63498854637146f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][0] += -1.2429687976837158f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][1] += 1.4835329055786133f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][0] += 1.025354266166687f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][1] += -1.032715916633606f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][0] += -0.8612805604934692f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][1] += 0.7673803567886353f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][0] += 1.0134145021438599f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][1] += -0.7070935964584351f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][0] += 1.4444584846496582f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][1] += -1.3293707370758057f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][0] += -1.1682406663894653f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][1] += 0.9483568072319031f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][0] += -1.6988931894302368f * x3[x_1][x_2][11];
			x4[x_out_1][x_out_2][1] += 1.827574372291565f * x3[x_1][x_2][11];
			x_1 = ix + 1;
			x_2 = jx + 0;
			x4[x_out_1][x_out_2][0] += 1.4380302429199219f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][1] += -1.632439136505127f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][0] += 0.7118520736694336f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][1] += -0.7636125087738037f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][0] += 1.2777174711227417f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][1] += -1.1640805006027222f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][0] += 0.19946521520614624f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][1] += 0.11687803268432617f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][0] += 1.0035611391067505f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][1] += -0.5327439308166504f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][0] += -1.8404643535614014f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][1] += 1.5287377834320068f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][0] += 1.5658280849456787f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][1] += -1.846311330795288f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][0] += -0.3321970999240875f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][1] += 0.19451382756233215f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][0] += 0.3236546516418457f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][1] += -0.020772404968738556f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][0] += -0.255141019821167f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][1] += 0.07719504088163376f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][0] += -1.8286834955215454f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][1] += 1.7686620950698853f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][0] += -1.7791264057159424f * x3[x_1][x_2][11];
			x4[x_out_1][x_out_2][1] += 1.4919602870941162f * x3[x_1][x_2][11];
			x_2 = jx + 1;
			x4[x_out_1][x_out_2][0] += 1.5871796607971191f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][1] += -1.6504075527191162f * x3[x_1][x_2][0];
			x4[x_out_1][x_out_2][0] += 1.3105922937393188f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][1] += -1.2756975889205933f * x3[x_1][x_2][1];
			x4[x_out_1][x_out_2][0] += 1.227034330368042f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][1] += -1.452986240386963f * x3[x_1][x_2][2];
			x4[x_out_1][x_out_2][0] += -0.6943020820617676f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][1] += 0.9953951835632324f * x3[x_1][x_2][3];
			x4[x_out_1][x_out_2][0] += 1.3031998872756958f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][1] += -1.5046194791793823f * x3[x_1][x_2][4];
			x4[x_out_1][x_out_2][0] += -1.5391290187835693f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][1] += 0.9699410200119019f * x3[x_1][x_2][5];
			x4[x_out_1][x_out_2][0] += 1.427099585533142f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][1] += -1.7612587213516235f * x3[x_1][x_2][6];
			x4[x_out_1][x_out_2][0] += -0.805960476398468f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][1] += 1.103092074394226f * x3[x_1][x_2][7];
			x4[x_out_1][x_out_2][0] += -0.2612428665161133f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][1] += 0.04796583205461502f * x3[x_1][x_2][8];
			x4[x_out_1][x_out_2][0] += -0.3838154375553131f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][1] += 0.6033097505569458f * x3[x_1][x_2][9];
			x4[x_out_1][x_out_2][0] += -1.3388670682907104f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][1] += 1.0003392696380615f * x3[x_1][x_2][10];
			x4[x_out_1][x_out_2][0] += -2.1253459453582764f * x3[x_1][x_2][11];
			x4[x_out_1][x_out_2][1] += 2.0291335582733154f * x3[x_1][x_2][11];
		}
	}
	static float x5[1][1][2] = {0};
	static float max5 = x4[0][0][0] > x4[0][0][1] ? x4[0][0][0] : x4[0][0][1];
	x5[0][0][0] = (float)exp(x4[0][0][0] - max5);
	x5[0][0][1] = (float)exp(x4[0][0][1] - max5);
	static float sum5;
	sum5 = x5[0][0][0] + x5[0][0][1];
	x5[0][0][0] /= sum5;
	x5[0][0][1] /= sum5;
	scores[0] = x5[0][0][0];
	scores[1] = x5[0][0][1];
	*res = 0;
	*res = scores[1] > scores[*res] ? 1 : *res;
	return 0;
}

#ifdef CNN_TEST
#include <stdio.h>
    
int main()
{
    int i, j, res;
    FILE *f = fopen("img.bin", "r");
    float x[16][16][1];
    float scores[2];
    for (j = 0; j < 16; j++)
        for (i = 0; i < 16; i++)
            for (int k = 0; k < 1; k++)
                fread(&x[j][i][k], sizeof(float), 1, f);
    fclose(f);
    res = 0;
    cnn(x, &res, scores);
    return res;
}
#endif
