#include "cnn.c"
#include <stdio.h>
#define size 16
int main()
{
    int i, j, res;
    FILE *f = fopen("img.bin", "r");
    float x[size][size][1];
    float scores[2];
    for (i = 0; i < size; i++)
        for (j = 0; j < size; j++)
            fread(&x[j][i][0], sizeof(float), 1, f);
    fclose(f);
    res = 0;
    cnn(x, &res, scores);
    return res;
}