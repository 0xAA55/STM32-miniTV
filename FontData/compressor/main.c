#include "miniz.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/stat.h>

#define PRIsize_t "zu"

void *my_alloc_func(void *opaque, size_t items, size_t size)
{
    void *addr = malloc(items * size);
    fprintf(stderr, "zalloc(%"PRIsize_t", %"PRIsize_t") = %p\n", items, size, addr);
    return addr;
}

void my_free_func(void *opaque, void *address)
{
    free(address);
    fprintf(stderr, "zfree(%p)\n", address);
}

int main(int argc, char** argv)
{
    FILE *fp = NULL;
    FILE *fp2 = NULL;
    struct stat st;
    uint8_t *srcfile_data = NULL;
    size_t srcfile_size;
    uint8_t *compress_buffer = NULL;
    size_t compressed_size;
    mz_stream stream;
    int status;
    int compressing = 0;

    if (argc < 3)
    {
        fprintf(stderr, "Usage: comp <input.bin> <output.bin>\n");
        fprintf(stderr, "The file <input.bin> will be compressed and generate <output.bin>\n");
        return 1;
    }

    if (stat(argv[1], &st))
    {
        perror("Trying to get file size");
        goto ErrorRet;
    }
    srcfile_size = st.st_size;
    if (srcfile_size > 0xFFFFFFFF)
    {
        fprintf(stderr, "miniz could only compress files smaller than 4GB.\n");
        goto ErrorRet;
    }

    fp = fopen(argv[1], "rb");
    if (!fp)
    {
        fprintf(stderr, "Could not open \"%s\" for read.\n", argv[1]);
        goto ErrorRet;
    }

    fp2 = fopen(argv[2], "wb");
    if (!fp2)
    {
        fprintf(stderr, "Could not open \"%s\" for write.\n", argv[2]);
        goto ErrorRet;
    }

    srcfile_data = malloc(srcfile_size);
    compress_buffer = malloc(srcfile_size);
    if (!srcfile_data || !compress_buffer)
    {
        perror("Trying to allocate memory for the file and allocate the compression buffer");
        goto ErrorRet;
    }

    if (fread(srcfile_data, srcfile_size, 1, fp) == 0)
    {
        perror("Trying to load the whole file into RAM");
        goto ErrorRet;
    }

    fclose(fp); fp = NULL;

    memset(&stream, 0, sizeof stream);

    stream.next_in = srcfile_data;
    stream.avail_in = (mz_uint32)srcfile_size;
    stream.next_out = compress_buffer;
    stream.avail_out = (mz_uint32)srcfile_size;

    status = mz_deflateInit(&stream, 10);
    if (status != MZ_OK)
    {
        fprintf(stderr, "mz_deflateInit() failed: %d\n", status);
        goto ErrorRet;
    }
    compressing = 1;

    status = mz_deflate(&stream, MZ_FINISH);
    if (status != MZ_STREAM_END)
    {
        mz_deflateEnd(&stream);
        if (status != MZ_OK)
        {
            fprintf(stderr, "mz_deflate() failed: %d\n", status);
            goto ErrorRet;
        }
    }

    compressed_size = stream.total_out;
    printf("SRC len: %"PRIsize_t", DST len: %"PRIsize_t"\n", srcfile_size, compressed_size);

    status = mz_deflateEnd(&stream);
    if (status != MZ_OK)
    {
        fprintf(stderr, "mz_deflate() failed: %d\n", status);
        goto ErrorRet;
    }
    compressing = 0;
    free(srcfile_data); srcfile_data = NULL;

    if (fwrite(compress_buffer, compressed_size, 1, fp2) == 0)
    {
        perror("Trying to save the compressed data");
        goto ErrorRet;
    }
    fclose(fp2); fp2 = NULL;
    free(compress_buffer); compress_buffer = NULL;
    return 0;
ErrorRet:
    if (compressing) mz_deflateEnd(&stream);
    if (fp) fclose(fp);
    if (fp2) fclose(fp2);
    free(srcfile_data);
    free(compress_buffer);
    return 2;
}
