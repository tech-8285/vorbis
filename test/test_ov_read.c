/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE OggVorbis SOFTWARE CODEC SOURCE CODE.   *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2007             *
 * by the Xiph.Org Foundation https://xiph.org/                     *
 *                                                                  *
 ********************************************************************

 function: unit tests for integer PCM packing in ov_read()/ov_read_filter()

 What is pinned down here is the documented contract of ov_read() (see
 doc/vorbisfile/ov_read.html), as implemented by ov_read_filter() in
 lib/vorbisfile.c:

   - The decoded float samples are quantized to the requested word size
     (word==1 -> 8 bit, word==2 -> 16 bit), optionally offset by half of
     the unsigned range, and packed with the channels interleaved in
     stream order (left, right, ...).
   - Values outside the representable range saturate: the positive peak
     becomes 127 (8 bit) / 32767 (16 bit) and the negative peak -128 /
     -32768, before any unsigned offset is applied.
   - 'length' is a byte capacity, not a request.  Only whole frames (one
     sample on every channel) are written, so the returned byte count is
     always a multiple of word*channels; the frames packed are exactly
     the frames consumed from the decoder (ov_pcm_tell() advances by that
     frame count); and no byte past the returned count is written.
   - If the capacity cannot hold one whole frame (0 < length <
     word*channels), the call reports OV_EINVAL, writes nothing, consumes
     nothing, and does not run the filter.  An incomplete frame is never
     presented as successfully decoded output, and the frame is still
     available to the next, large enough read.

 The audio material is generated locally by this program with
 libvorbisenc -- no network access and no external files.  Each channel
 gets its own frequency, phase and peak amplitude, so a channel swap or a
 broken interleave shows up as a large error instead of a plausible
 sample.  Expected integer values are taken from an independent float
 decode of the same file via ov_read_float(), i.e. the integer packing is
 checked against the float PCM it converts, not against whatever the
 currently built library happens to emit.

 Set OV_READ_TEST_KEEP=1 to keep the generated .ogg files.

 ********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include <vorbis/codec.h>
#include <vorbis/vorbisenc.h>
#include <vorbis/vorbisfile.h>

#define ARRAY_LEN(x)     (sizeof(x)/sizeof((x)[0]))

#define MATERIAL_RATE    44100
#define MATERIAL_FRAMES  1537       /* not a block size, not a packet multiple */
#define MATERIAL_QUALITY 0.4f

#define IO_CHUNK         512        /* frames handed to the encoder at a time */
#define REF_CHUNK        1024       /* frames pulled from ov_read_float() */
#define BUF_BYTES        16384      /* output buffer for every ov_read() call */

#define UNWRITTEN        0x5a       /* untouched-buffer sentinel */
#define REJECTED         0xa5       /* untouched-buffer sentinel on reject paths */

/* ------------------------------------------------------------------ */
/* reporting                                                          */

static int g_failures;
static int g_reports;

static void
report_failure (const char *file, int line, const char *fmt, ...)
{
  va_list ap;

  g_failures++;
  if (g_reports >= 40)
    return;
  g_reports++;
  printf ("FAIL %s:%d: ", file, line);
  va_start (ap, fmt);
  vprintf (fmt, ap);
  va_end (ap);
  printf ("\n");
  fflush (stdout);
}

#define FAILF(...)  report_failure (__FILE__, __LINE__, __VA_ARGS__)

/* ------------------------------------------------------------------ */
/* locally generated material                                         */

/* Channel-dependent tone: distinct frequency, phase and peak level per
   channel, all well below full scale.  Saturation is only reached via
   the gain filter used by the saturation test. */
static double
material_sample (int channel, long frame)
{
  const double two_pi = 6.283185307179586476925286766559;
  double freq  = 180.0 + 90.0 * (double)channel;
  double phase = 0.61 * (double)channel;
  double amp   = 0.78 - 0.04 * (double)channel;

  return amp * sin (two_pi * freq * (double)frame / (double)MATERIAL_RATE + phase);
}

static void
write_pages (FILE *file, ogg_stream_state *os, int *eos)
{
  ogg_page og;
  int result;

  while (!*eos) {
    result = ogg_stream_pageout (os, &og);
    if (result == 0)
      return;
    fwrite (og.header, 1, (size_t)og.header_len, file);
    fwrite (og.body, 1, (size_t)og.body_len, file);
    if (ogg_page_eos (&og))
      *eos = 1;
  }
}

static int
write_material (const char *path, int channels, long frames)
{
  FILE *file;
  ogg_stream_state os;
  ogg_page og;
  ogg_packet op;
  vorbis_info vi;
  vorbis_comment vc;
  vorbis_dsp_state vd;
  vorbis_block vb;
  int eos = 0;
  long pos;
  int ret;

  if (!(file = fopen (path, "wb")))
    return 0;

  vorbis_info_init (&vi);
  ret = vorbis_encode_init_vbr (&vi, channels, MATERIAL_RATE, MATERIAL_QUALITY);
  if (ret) {
    printf ("vorbis_encode_init_vbr(%d channels) returned %d\n", channels, ret);
    fclose (file);
    return 0;
  }
  vorbis_comment_init (&vc);
  vorbis_comment_add_tag (&vc, "ENCODER", "test/test_ov_read.c");
  vorbis_analysis_init (&vd, &vi);
  vorbis_block_init (&vd, &vb);
  ogg_stream_init (&os, 12345678);

  {
    ogg_packet header;
    ogg_packet header_comm;
    ogg_packet header_code;

    vorbis_analysis_headerout (&vd, &vc, &header, &header_comm, &header_code);
    ogg_stream_packetin (&os, &header);
    ogg_stream_packetin (&os, &header_comm);
    ogg_stream_packetin (&os, &header_code);

    /* start the audio on a fresh page */
    while (!eos) {
      int result = ogg_stream_flush (&os, &og);
      if (result == 0)
        break;
      fwrite (og.header, 1, (size_t)og.header_len, file);
      fwrite (og.body, 1, (size_t)og.body_len, file);
    }
  }

  for (pos = 0; pos < frames; pos += IO_CHUNK) {
    long n = frames - pos;
    float **buffer;
    int i;

    if (n > IO_CHUNK)
      n = IO_CHUNK;
    buffer = vorbis_analysis_buffer (&vd, (int)n);
    for (i = 0; i < channels; i++) {
      long j;
      for (j = 0; j < n; j++)
        buffer[i][j] = (float)material_sample (i, pos + j);
    }
    vorbis_analysis_wrote (&vd, (int)n);

    while (vorbis_analysis_blockout (&vd, &vb) == 1) {
      vorbis_analysis (&vb, NULL);
      vorbis_bitrate_addblock (&vb);
      while (vorbis_bitrate_flushpacket (&vd, &op)) {
        ogg_stream_packetin (&os, &op);
        write_pages (file, &os, &eos);
      }
    }
  }

  vorbis_analysis_wrote (&vd, 0);
  while (vorbis_analysis_blockout (&vd, &vb) == 1) {
    vorbis_analysis (&vb, NULL);
    vorbis_bitrate_addblock (&vb);
    while (vorbis_bitrate_flushpacket (&vd, &op)) {
      ogg_stream_packetin (&os, &op);
      write_pages (file, &os, &eos);
    }
  }

  ogg_stream_clear (&os);
  vorbis_block_clear (&vb);
  vorbis_dsp_clear (&vd);
  vorbis_comment_clear (&vc);
  vorbis_info_clear (&vi);
  fclose (file);

  return 1;
}

/* Whole-file float decode used as the value oracle. */
static int
decode_reference (const char *path, int channels, long frames, float **ref)
{
  OggVorbis_File vf;
  long got = 0;
  int ok = 1;

  if (ov_fopen (path, &vf) < 0) {
    FAILF ("cannot open '%s'", path);
    return 0;
  }
  if (ov_info (&vf, -1)->channels != channels) {
    FAILF ("'%s' decodes as %d channels, expected %d",
           path, ov_info (&vf, -1)->channels, channels);
    ok = 0;
  }
  if (ov_pcm_total (&vf, -1) != frames) {
    FAILF ("'%s' holds %lld frames, expected %ld",
           path, (long long)ov_pcm_total (&vf, -1), frames);
    ok = 0;
  }

  while (ok) {
    float **pcm;
    long n = ov_read_float (&vf, &pcm, REF_CHUNK, NULL);
    int i;

    if (n == 0)
      break;
    if (n < 0) {
      FAILF ("ov_read_float() failed on '%s' (%ld)", path, n);
      ok = 0;
      break;
    }
    if (got + n > frames) {
      FAILF ("ov_read_float() overran '%s'", path);
      ok = 0;
      break;
    }
    for (i = 0; i < channels; i++)
      memcpy (ref[i] + got, pcm[i], (size_t)n * sizeof (float));
    got += n;
  }

  if (ok && got != frames) {
    FAILF ("ov_read_float() returned %ld of %ld frames from '%s'",
           got, frames, path);
    ok = 0;
  }

  ov_clear (&vf);
  return ok;
}

/* ------------------------------------------------------------------ */
/* integer oracle                                                     */

/* Rounding in ov_read_filter() is done by vorbis_ftoi(), whose exact
   tie behaviour depends on the platform and FPU mode.  Instead of baking
   in one rounder, every plausible result for a sample is computed (the
   product in float and in double precision, rounded half away from zero
   and half to even) and the packed sample must fall inside that window.
   The window can only ever be one LSB wide at an exact tie, so a
   systematic off-by-one is still a failure. */
typedef struct {
  long lo;
  long hi;
  int  sat;   /* +1 clamps at the positive peak, -1 at the negative peak */
} quantized;

static long
round_half_away (double v)
{
  return (long)(v >= 0.0 ? floor (v + 0.5) : ceil (v - 0.5));
}

static long
round_half_even (double v)
{
  double f = floor (v);
  double frac = v - f;

  if (frac > 0.5)
    return (long)(f + 1.0);
  if (frac < 0.5)
    return (long)f;
  return ((long)f & 1) ? (long)(f + 1.0) : (long)f;
}

static void
quantize_sample (float x, int word, quantized *q)
{
  double scale = (word == 1) ? 128.0 : 32768.0;
  long limit_lo = (word == 1) ? -128 : -32768;
  long limit_hi = (word == 1) ?  127 :  32767;
  float  fprod = (float)x * (float)scale;
  double dprod = (double)x * scale;
  long cand[4];
  int i;

  cand[0] = round_half_away ((double)fprod);
  cand[1] = round_half_even ((double)fprod);
  cand[2] = round_half_away (dprod);
  cand[3] = round_half_even (dprod);

  q->lo = q->hi = cand[0];
  for (i = 1; i < 4; i++) {
    if (cand[i] < q->lo) q->lo = cand[i];
    if (cand[i] > q->hi) q->hi = cand[i];
  }
  q->sat = 0;

  if (q->lo > limit_hi) {
    q->lo = q->hi = limit_hi;
    q->sat = 1;
  } else if (q->hi < limit_lo) {
    q->lo = q->hi = limit_lo;
    q->sat = -1;
  } else {
    if (q->lo < limit_lo) q->lo = limit_lo;
    if (q->hi > limit_hi) q->hi = limit_hi;
  }
}

/* Unpack one packed sample back to a canonical signed integer in
   [-128,127] / [-32768,32767], for any byte order and signedness. */
static long
canonical_sample (const unsigned char *p, int word, int bigendianp, int sgned)
{
  if (word == 1) {
    long v = p[0];
    return sgned ? (v >= 128 ? v - 256 : v) : v - 128;
  } else {
    long hi = p[bigendianp ? 0 : 1];
    long lo = p[bigendianp ? 1 : 0];
    long v = (hi << 8) | lo;
    return sgned ? (v >= 32768 ? v - 65536 : v) : v - 32768;
  }
}

typedef struct {
  long checked;
  long pos_sat;
  long neg_sat;
} compare_stats;

/* Compare 'frames' whole frames of interleaved output against the
   reference, starting at reference frame 'frame0' and at sample
   'byte0' of the packed buffer.  Returns the number of bad samples. */
static long
compare_frames (const float *const *ref, int channels,
                long frame0, long frames,
                const unsigned char *buf, size_t byte0,
                int word, int bigendianp, int sgned,
                compare_stats *stats)
{
  const size_t fs = (size_t)word * (size_t)channels;
  long bad = 0;
  long j;

  for (j = 0; j < frames; j++) {
    int i;
    for (i = 0; i < channels; i++) {
      const unsigned char *p = buf + byte0 + (size_t)j * fs + (size_t)i * (size_t)word;
      long canon = canonical_sample (p, word, bigendianp, sgned);
      quantized q;

      quantize_sample (ref[i][frame0 + j], word, &q);
      stats->checked++;

      if (q.sat > 0) {
        stats->pos_sat++;
        if (canon != q.lo && bad++ < 8)
          FAILF ("positive peak not saturated: frame %ld ch %d got %ld, "
                 "expected %ld", frame0 + j, i, canon, q.lo);
      } else if (q.sat < 0) {
        stats->neg_sat++;
        if (canon != q.lo && bad++ < 8)
          FAILF ("negative peak not saturated: frame %ld ch %d got %ld, "
                 "expected %ld", frame0 + j, i, canon, q.lo);
      } else if (canon < q.lo || canon > q.hi) {
        if (bad++ < 8)
          FAILF ("value error: frame %ld ch %d got %ld, expected %ld..%ld "
                 "(float %g)", frame0 + j, i, canon, q.lo, q.hi,
                 (double)ref[i][frame0 + j]);
      }
    }
  }

  return bad;
}

/* ------------------------------------------------------------------ */
/* gain filter (drives the peaks into saturation)                     */

typedef struct {
  float gain;
} gain_params;

static void
apply_gain (float **pcm, long channels, long samples, void *filter_param)
{
  const gain_params *g = (const gain_params *)filter_param;
  long i, j;

  for (i = 0; i < channels; i++)
    for (j = 0; j < samples; j++)
      pcm[i][j] *= g->gain;
}

/* ------------------------------------------------------------------ */
/* helpers                                                            */

static int
buffer_untouched (const unsigned char *buf, size_t from, size_t to, int sentinel)
{
  size_t k;

  for (k = from; k < to; k++)
    if (buf[k] != (unsigned char)sentinel)
      return 0;
  return 1;
}

static int
open_or_fail (const char *path, OggVorbis_File *vf)
{
  if (ov_fopen (path, vf) < 0) {
    FAILF ("cannot open '%s'", path);
    return 0;
  }
  return 1;
}

/* ------------------------------------------------------------------ */
/* test A: capacity -> frame count, and values of the emitted frames  */

/* Walk one stream through capacities that do not hold a frame, hold
   none, hold exactly one frame, hold whole frames plus a remainder and
   hold many frames.  After every call:
     - a capacity below one frame yields OV_EINVAL, no write, no consume;
     - a successful read returns a whole number of frames that fits the
       capacity, writes nothing past the returned count, and advances
       ov_pcm_tell() by exactly that many frames;
     - the packed samples equal the reference for the frames consumed. */
static int
test_capacity_and_values (const char *path, int channels, long total,
                          const float *const *ref)
{
  static unsigned char buf[BUF_BYTES];
  int words[2] = { 1, 2 };
  int ok = 1;
  size_t w;

  for (w = 0; w < ARRAY_LEN (words); w++) {
    int word = words[w];
    int fs = word * channels;
    int caps[11];
    size_t nc = 0, c;
    OggVorbis_File vf;
    long cursor = 0;

    caps[nc++] = 0;
    caps[nc++] = 1;
    caps[nc++] = fs - 1;
    caps[nc++] = fs;
    caps[nc++] = fs + 1;
    caps[nc++] = 3 * fs - 1;
    caps[nc++] = 3 * fs;
    caps[nc++] = 3 * fs + fs / 2;
    caps[nc++] = 7 * fs + 1;
    caps[nc++] = 4093;
    caps[nc++] = 8192;

    for (c = 0; c < nc; c++) {
      if (caps[c] >= BUF_BYTES) {
        FAILF ("test bug: capacity %d exceeds the test buffer", caps[c]);
        return 0;
      }
    }

    if (!open_or_fail (path, &vf))
      return 0;
    if (ov_pcm_tell (&vf) != 0) {
      FAILF ("ch=%d word=%d: first frame is not at pcm position 0",
             channels, word);
      ok = 0;
    }

    for (c = 0; c < nc; c++) {
      int cap = caps[c];
      long before, after, ret, frames;
      compare_stats st;

      memset (buf, UNWRITTEN, sizeof (buf));
      before = (long)ov_pcm_tell (&vf);
      ret = ov_read (&vf, (char *)buf, cap, 0, word, 1, NULL);
      after = (long)ov_pcm_tell (&vf);

      if (cap < fs) {
        if (ret != OV_EINVAL) {
          FAILF ("ch=%d word=%d cap=%d (< %d): expected OV_EINVAL, got %ld",
                 channels, word, cap, fs, ret);
          ok = 0;
        }
        if (after != before) {
          FAILF ("ch=%d word=%d cap=%d: rejected call consumed %ld frames",
                 channels, word, cap, after - before);
          ok = 0;
        }
        if (!buffer_untouched (buf, 0, sizeof (buf), UNWRITTEN)) {
          FAILF ("ch=%d word=%d cap=%d: rejected call wrote to the buffer",
                 channels, word, cap);
          ok = 0;
        }
        continue;
      }

      if (ret < 0) {
        FAILF ("ch=%d word=%d cap=%d: unexpected error %ld",
               channels, word, cap, ret);
        ok = 0;
        continue;
      }
      if (ret == 0) {
        if (after != total || cursor != total) {
          FAILF ("ch=%d word=%d cap=%d: EOF at frame %ld of %ld",
                 channels, word, cap, after, total);
          ok = 0;
        }
        continue;
      }

      frames = ret / fs;
      if (ret % fs) {
        FAILF ("ch=%d word=%d cap=%d: %ld bytes is not a whole number of "
               "%d-byte frames", channels, word, cap, ret, fs);
        ok = 0;
        continue;
      }
      if (ret > cap) {
        FAILF ("ch=%d word=%d cap=%d: returned %ld bytes past the capacity",
               channels, word, cap, ret);
        ok = 0;
      }
      if (frames > cap / fs) {
        FAILF ("ch=%d word=%d cap=%d: %ld frames do not fit the capacity",
               channels, word, cap, frames);
        ok = 0;
      }
      if (after - before != frames) {
        FAILF ("ch=%d word=%d cap=%d: packed %ld frames but pcm position "
               "moved by %ld", channels, word, cap, frames, after - before);
        ok = 0;
      }
      if (after != cursor + frames) {
        FAILF ("ch=%d word=%d cap=%d: frame counting drifted (%ld != %ld)",
               channels, word, cap, after, cursor + frames);
        ok = 0;
      }
      if (cursor + frames > total) {
        FAILF ("ch=%d word=%d cap=%d: read past the end of the stream",
               channels, word, cap);
        ok = 0;
        continue;
      }
      memset (&st, 0, sizeof (st));
      if (compare_frames (ref, channels, cursor, frames, buf, 0,
                          word, 0, 1, &st)) {
        FAILF ("ch=%d word=%d cap=%d: value mismatch in frames %ld..%ld",
               channels, word, cap, cursor, cursor + frames - 1);
        ok = 0;
      }
      if (!buffer_untouched (buf, (size_t)ret, sizeof (buf), UNWRITTEN)) {
        FAILF ("ch=%d word=%d cap=%d: wrote past the returned %ld bytes",
               channels, word, cap, ret);
        ok = 0;
      }
      cursor += frames;
    }

    /* drain the rest in frame-aligned chunks and keep checking values */
    for (;;) {
      long ret = ov_read (&vf, (char *)buf, fs * 64, 0, word, 1, NULL);
      compare_stats st;

      if (ret == 0)
        break;
      if (ret < 0) {
        FAILF ("ch=%d word=%d: error %ld while draining", channels, word, ret);
        ok = 0;
        break;
      }
      if (ret % fs) {
        FAILF ("ch=%d word=%d: %ld bytes while draining is not whole frames",
               channels, word, ret);
        ok = 0;
        break;
      }
      if (cursor + ret / fs > total) {
        FAILF ("ch=%d word=%d: drain overran the stream", channels, word);
        ok = 0;
        break;
      }
      memset (&st, 0, sizeof (st));
      if (compare_frames (ref, channels, cursor, ret / fs, buf, 0,
                          word, 0, 1, &st)) {
        FAILF ("ch=%d word=%d: value mismatch while draining at frame %ld",
               channels, word, cursor);
        ok = 0;
      }
      cursor += ret / fs;
    }

    if (cursor != total) {
      FAILF ("ch=%d word=%d: consumed %ld of %ld frames",
             channels, word, cursor, total);
      ok = 0;
    }
    if (ov_pcm_tell (&vf) != total) {
      FAILF ("ch=%d word=%d: pcm position is %lld after %ld frames",
             channels, word, (long long)ov_pcm_tell (&vf), total);
      ok = 0;
    }

    ov_clear (&vf);
  }

  return ok;
}

/* ------------------------------------------------------------------ */
/* test B: byte order / signedness / word size matrix                  */

/* Decode the whole stream for each of the four packings per word size.
   All of them must yield the same canonical samples, and those samples
   must agree with the float reference.  This makes the packing itself,
   not one code path, the contract. */
static int
test_format_matrix (const char *path, int channels, long total,
                    const float *const *ref)
{
  static unsigned char buf[BUF_BYTES];
  int words[2] = { 1, 2 };
  int ok = 1;
  size_t w;

  for (w = 0; w < ARRAY_LEN (words); w++) {
    int word = words[w];
    int fs = word * channels;
    long *baseline;
    int big, sgn;
    int first = 1;

    baseline = (long *)malloc ((size_t)total * (size_t)channels * sizeof (long));
    if (!baseline) {
      FAILF ("out of memory");
      return 0;
    }

    for (big = 0; big < 2; big++) {
      for (sgn = 0; sgn < 2; sgn++) {
        OggVorbis_File vf;
        long cursor = 0;

        if (!open_or_fail (path, &vf)) {
          free (baseline);
          return 0;
        }
        for (;;) {
          long ret = ov_read (&vf, (char *)buf, (int)sizeof (buf),
                              big, word, sgn, NULL);
          long frames, j;
          int i;

          if (ret == 0)
            break;
          if (ret < 0) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: read error %ld",
                   channels, word, big, sgn, ret);
            ok = 0;
            break;
          }
          if (ret % fs) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: %ld bytes is not whole frames",
                   channels, word, big, sgn, ret);
            ok = 0;
            break;
          }
          frames = ret / fs;
          if (cursor + frames > total) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: overran the stream",
                   channels, word, big, sgn);
            ok = 0;
            break;
          }
          for (j = 0; j < frames; j++) {
            for (i = 0; i < channels; i++) {
              long canon = canonical_sample (buf + (j * channels + i) * word,
                                             word, big, sgn);
              long idx = (cursor + j) * channels + i;
              quantized q;

              quantize_sample (ref[i][cursor + j], word, &q);
              if (first) {
                baseline[idx] = canon;
                if (q.sat != 0) {
                  if (canon != q.lo) {
                    FAILF ("ch=%d word=%d big=%d sgn=%d: saturation mismatch "
                           "at frame %ld ch %d", channels, word, big, sgn,
                           cursor + j, i);
                    ok = 0;
                  }
                } else if (canon < q.lo || canon > q.hi) {
                  FAILF ("ch=%d word=%d big=%d sgn=%d: value mismatch at frame "
                         "%ld ch %d (got %ld, expected %ld..%ld)", channels,
                         word, big, sgn, cursor + j, i, canon, q.lo, q.hi);
                  ok = 0;
                }
              } else if (canon != baseline[idx]) {
                FAILF ("ch=%d word=%d big=%d sgn=%d: packing disagrees with "
                       "the first one at frame %ld ch %d (got %ld, "
                       "baseline %ld)", channels, word, big, sgn, cursor + j,
                       i, canon, baseline[idx]);
                ok = 0;
              }
            }
          }
          cursor += frames;
        }
        if (cursor != total) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: decoded %ld of %ld frames",
                 channels, word, big, sgn, cursor, total);
          ok = 0;
        }
        if (ov_pcm_tell (&vf) != total) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: pcm position %lld after %ld "
                 "frames", channels, word, big, sgn,
                 (long long)ov_pcm_tell (&vf), total);
          ok = 0;
        }
        ov_clear (&vf);
        first = 0;
      }
    }

    free (baseline);
  }

  return ok;
}

/* ------------------------------------------------------------------ */
/* test C: peak saturation on exactly the frames reported as consumed  */

/* ov_read_filter() with a gain that drives both peaks into saturation.
   Capacities are frame aligned except for a few that must be rejected.
   Every sample checked for saturation belongs to the frames counted as
   consumed, and the returned byte count is exactly frames*word*channels;
   the filter must not run on data that the call does not deliver. */
static int
test_peak_saturation (const char *path, int channels, long total,
                      const float *const *ref)
{
  static const int frame_caps[] = { 1, 3, 7, 13, 64 };
  static unsigned char buf[BUF_BYTES];
  gain_params gp;
  int words[2] = { 1, 2 };
  int ok = 1;
  size_t w;

  gp.gain = 2.2f;

  for (w = 0; w < ARRAY_LEN (words); w++) {
    int word = words[w];
    int fs = word * channels;
    int big, sgn;

    for (big = 0; big < 2; big++) {
      for (sgn = 0; sgn < 2; sgn++) {
        OggVorbis_File vf;
        compare_stats st;
        float **scaled;
        long cursor = 0;
        size_t c = 0;
        int i;

        memset (&st, 0, sizeof (st));
        scaled = (float **)malloc ((size_t)channels * sizeof (float *));
        if (!scaled) {
          FAILF ("out of memory");
          return 0;
        }
        for (i = 0; i < channels; i++) {
          scaled[i] = (float *)malloc ((size_t)64 * sizeof (float));
          if (!scaled[i]) {
            FAILF ("out of memory");
            return 0;
          }
        }

        if (!open_or_fail (path, &vf)) {
          return 0;
        }

        /* Capacities that cannot hold a frame must be rejected before
           the filter ever runs, and must not consume anything. */
        {
          int bad_cap;
          for (bad_cap = 0; bad_cap < fs; bad_cap++) {
            long before = (long)ov_pcm_tell (&vf);
            long ret;

            memset (buf, REJECTED, sizeof (buf));
            ret = ov_read_filter (&vf, (char *)buf, bad_cap, big, word, sgn,
                                  NULL, apply_gain, &gp);
            if (ret != OV_EINVAL) {
              FAILF ("ch=%d word=%d big=%d sgn=%d: filtered cap=%d expected "
                     "OV_EINVAL, got %ld", channels, word, big, sgn, bad_cap,
                     ret);
              ok = 0;
            }
            if ((long)ov_pcm_tell (&vf) != before) {
              FAILF ("ch=%d word=%d big=%d sgn=%d: filtered reject consumed "
                     "frames", channels, word, big, sgn);
              ok = 0;
            }
            if (!buffer_untouched (buf, 0, sizeof (buf), REJECTED)) {
              FAILF ("ch=%d word=%d big=%d sgn=%d: filtered reject wrote",
                     channels, word, big, sgn);
              ok = 0;
            }
          }
        }

        for (;;) {
          int cap = frame_caps[c % ARRAY_LEN (frame_caps)] * fs;
          long before = (long)ov_pcm_tell (&vf);
          long ret = ov_read_filter (&vf, (char *)buf, cap, big, word, sgn,
                                     NULL, apply_gain, &gp);
          long after = (long)ov_pcm_tell (&vf);
          long frames;
          long j;

          c++;
          if (ret == 0)
            break;
          if (ret < 0) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: filtered read error %ld",
                   channels, word, big, sgn, ret);
            ok = 0;
            break;
          }
          if (ret % fs) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: %ld filtered bytes is not "
                   "whole frames", channels, word, big, sgn, ret);
            ok = 0;
            break;
          }
          frames = ret / fs;
          if (after - before != frames) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: %ld frames packed but pcm "
                   "position moved by %ld", channels, word, big, sgn,
                   frames, after - before);
            ok = 0;
          }
          if (frames > cap / fs) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: %ld frames exceed capacity",
                   channels, word, big, sgn, frames);
            ok = 0;
          }
          if (cursor + frames > total) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: overran the stream",
                   channels, word, big, sgn);
            ok = 0;
            break;
          }
          if (frames > 64) {
            FAILF ("test bug: %ld frames exceed the scaled buffer", frames);
            ok = 0;
            break;
          }

          /* The filter is part of the contract: compare against the
             reference with the same gain applied.  A filter that ran on
             frames it did not deliver would show up on the next call as
             a second application of the gain. */
          for (i = 0; i < channels; i++)
            for (j = 0; j < frames; j++)
              scaled[i][j] = ref[i][cursor + j] * gp.gain;

          if (compare_frames ((const float *const *)scaled, channels,
                              0, frames, buf, 0, word, big, sgn, &st)) {
            FAILF ("ch=%d word=%d big=%d sgn=%d: peak mismatch in frames "
                   "%ld..%ld", channels, word, big, sgn, cursor,
                   cursor + frames - 1);
            ok = 0;
          }
          cursor += frames;
        }

        if (cursor != total) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: consumed %ld of %ld frames",
                 channels, word, big, sgn, cursor, total);
          ok = 0;
        }
        if (st.checked != total * channels) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: checked %ld of %ld samples",
                 channels, word, big, sgn, st.checked, total * channels);
          ok = 0;
        }
        if (st.pos_sat <= 0 || st.neg_sat <= 0) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: material did not reach both "
                 "peaks (pos=%ld neg=%ld)", channels, word, big, sgn,
                 st.pos_sat, st.neg_sat);
          ok = 0;
        }
        if (st.pos_sat + st.neg_sat > st.checked) {
          FAILF ("ch=%d word=%d big=%d sgn=%d: impossible saturation counts",
                 channels, word, big, sgn);
          ok = 0;
        }

        for (i = 0; i < channels; i++)
          free (scaled[i]);
        free (scaled);
        ov_clear (&vf);
      }
    }
  }

  return ok;
}

/* ------------------------------------------------------------------ */
/* test D: no incomplete frame is ever reported as output             */

/* Exhaust every capacity from 0 to one frame short: each must be
   rejected without consuming or writing, and the first frame must still
   be delivered by the next valid read.  A capacity of k frames plus a
   remainder must deliver exactly k frames.  Reading one frame at a time
   must reproduce the stream exactly. */
static int
test_partial_frame_rejection (const char *path, int channels, long total,
                              const float *const *ref)
{
  static unsigned char buf[BUF_BYTES];
  int words[2] = { 1, 2 };
  int ok = 1;
  size_t w;

  for (w = 0; w < ARRAY_LEN (words); w++) {
    int word = words[w];
    int fs = word * channels;
    OggVorbis_File vf;
    int len;

    if (!open_or_fail (path, &vf))
      return 0;

    for (len = 0; len < fs; len++) {
      long before, after, ret;

      memset (buf, REJECTED, sizeof (buf));
      before = (long)ov_pcm_tell (&vf);
      ret = ov_read (&vf, (char *)buf, len, 0, word, 1, NULL);
      after = (long)ov_pcm_tell (&vf);

      if (ret != OV_EINVAL) {
        FAILF ("ch=%d word=%d len=%d: expected OV_EINVAL, got %ld",
               channels, word, len, ret);
        ok = 0;
      }
      if (after != before) {
        FAILF ("ch=%d word=%d len=%d: rejected call moved pcm position by %ld",
               channels, word, len, after - before);
        ok = 0;
      }
      if (!buffer_untouched (buf, 0, sizeof (buf), REJECTED)) {
        FAILF ("ch=%d word=%d len=%d: rejected call wrote to the buffer",
               channels, word, len);
        ok = 0;
      }
    }

    /* Nothing was consumed: the first frames are still there. */
    {
      long ret;
      compare_stats st;

      memset (&st, 0, sizeof (st));
      memset (buf, UNWRITTEN, sizeof (buf));
      ret = ov_read (&vf, (char *)buf, 2 * fs, 0, word, 1, NULL);
      if (ret != 2 * fs) {
        FAILF ("ch=%d word=%d: after rejections expected %d bytes, got %ld",
               channels, word, 2 * fs, ret);
        ok = 0;
      } else if (compare_frames (ref, channels, 0, 2, buf, 0, word, 0, 1, &st)) {
        FAILF ("ch=%d word=%d: first frames changed after rejected calls",
               channels, word);
        ok = 0;
      }
    }

    /* Whole frames plus a remainder: only the whole frames come back.
       The stream has far more than three frames available here. */
    {
      long before = (long)ov_pcm_tell (&vf);
      long ret = ov_read (&vf, (char *)buf, 3 * fs + fs - 1, 0, word, 1, NULL);
      long after = (long)ov_pcm_tell (&vf);
      compare_stats st;

      memset (&st, 0, sizeof (st));
      if (ret != 3 * fs) {
        FAILF ("ch=%d word=%d: capacity of 3 frames + %d bytes returned %ld "
               "bytes instead of %d", channels, word, fs - 1, ret, 3 * fs);
        ok = 0;
      }
      if (after - before != 3) {
        FAILF ("ch=%d word=%d: %ld frames consumed for a 3-frame+remainder "
               "capacity", channels, word, after - before);
        ok = 0;
      }
      if (ret == 3 * fs &&
          compare_frames (ref, channels, 2, 3, buf, 0, word, 0, 1, &st)) {
        FAILF ("ch=%d word=%d: wrong samples after partial-frame rejection",
               channels, word);
        ok = 0;
      }
    }

    /* One frame at a time must reproduce the stream exactly. */
    {
      long cursor = 5;
      long ret;

      while (cursor < total &&
             (ret = ov_read (&vf, (char *)buf, fs, 0, word, 1, NULL)) > 0) {
        compare_stats st;

        memset (&st, 0, sizeof (st));
        if (ret != fs) {
          FAILF ("ch=%d word=%d: one-frame read returned %ld bytes",
                 channels, word, ret);
          ok = 0;
          break;
        }
        if (compare_frames (ref, channels, cursor, 1, buf, 0, word, 0, 1, &st)) {
          FAILF ("ch=%d word=%d: one-frame read mismatch at frame %ld",
                 channels, word, cursor);
          ok = 0;
          break;
        }
        cursor++;
      }
    }

    ov_clear (&vf);
  }

  return ok;
}

/* ------------------------------------------------------------------ */
/* test E: ov_read() is ov_read_filter() with a no-op filter          */

static int
test_identity_filter (const char *path, int channels, long total)
{
  static unsigned char plain[BUF_BYTES];
  static unsigned char filtered[BUF_BYTES];
  gain_params gp;
  OggVorbis_File a, b;
  int ok = 1;

  gp.gain = 1.0f;

  if (ov_fopen (path, &a) < 0) {
    FAILF ("cannot open '%s'", path);
    return 0;
  }
  if (ov_fopen (path, &b) < 0) {
    FAILF ("cannot open '%s'", path);
    ov_clear (&a);
    return 0;
  }

  for (;;) {
    long ra = ov_read (&a, (char *)plain, (int)sizeof (plain), 1, 2, 1, NULL);
    long rb = ov_read_filter (&b, (char *)filtered, (int)sizeof (filtered), 1, 2,
                              1, NULL, apply_gain, &gp);

    if (ra != rb) {
      FAILF ("ch=%d: plain read returned %ld bytes, identity filter %ld",
             channels, ra, rb);
      ok = 0;
      break;
    }
    if (ra == 0)
      break;
    if (ra < 0) {
      FAILF ("ch=%d: read error %ld", channels, ra);
      ok = 0;
      break;
    }
    if (memcmp (plain, filtered, (size_t)ra)) {
      FAILF ("ch=%d: identity filter changed the integer output", channels);
      ok = 0;
      break;
    }
  }

  if (ok && (ov_pcm_tell (&a) != total || ov_pcm_tell (&b) != total)) {
    FAILF ("ch=%d: identity filter changed the consumed frame count", channels);
    ok = 0;
  }

  ov_clear (&a);
  ov_clear (&b);
  return ok;
}

/* ------------------------------------------------------------------ */

typedef struct {
  const char *name;
  int (*run) (const char *path, int channels, long total,
              const float *const *ref);
  int needs_ref;
} test_entry;

static int
run_identity_adapter (const char *path, int channels, long total,
                      const float *const *ref)
{
  (void)ref;
  return test_identity_filter (path, channels, total);
}

int
main (void)
{
  static const int channel_counts[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  static const test_entry tests[] = {
    { "capacity and values",  test_capacity_and_values,    1 },
    { "format matrix",        test_format_matrix,          1 },
    { "peak saturation",      test_peak_saturation,        1 },
    { "partial frames",       test_partial_frame_rejection, 1 },
    { "identity filter",      run_identity_adapter,        0 }
  };
  int keep = getenv ("OV_READ_TEST_KEEP") != NULL;
  char path[128];
  size_t t, c;

  printf ("Testing integer PCM packing in ov_read()/ov_read_filter()\n");

  for (c = 0; c < ARRAY_LEN (channel_counts); c++) {
    int channels = channel_counts[c];
    float **ref;
    int i;

    snprintf (path, sizeof (path), "ov_read_material_%dch.ogg", channels);
    if (!write_material (path, channels, MATERIAL_FRAMES)) {
      FAILF ("could not write test material for %d channels", channels);
      return 1;
    }

    ref = (float **)malloc ((size_t)channels * sizeof (float *));
    if (!ref) {
      FAILF ("out of memory");
      return 1;
    }
    ref[0] = (float *)malloc ((size_t)channels * MATERIAL_FRAMES * sizeof (float));
    if (!ref[0]) {
      FAILF ("out of memory");
      return 1;
    }
    for (i = 1; i < channels; i++)
      ref[i] = ref[0] + (size_t)i * MATERIAL_FRAMES;

    if (!decode_reference (path, channels, MATERIAL_FRAMES, ref)) {
      FAILF ("%d channel reference decode failed", channels);
      return 1;
    }

    for (t = 0; t < ARRAY_LEN (tests); t++) {
      printf ("  %d channel%s: %-20s ", channels, channels == 1 ? "" : "s",
              tests[t].name);
      fflush (stdout);
      if (tests[t].needs_ref)
        tests[t].run (path, channels, MATERIAL_FRAMES,
                      (const float *const *)ref);
      else
        tests[t].run (path, channels, MATERIAL_FRAMES, NULL);
      printf ("done\n");
    }

    free (ref[0]);
    free (ref);

    if (!keep)
      remove (path);
  }

  if (g_failures) {
    printf ("%d failure%s\n", g_failures, g_failures == 1 ? "" : "s");
    return 1;
  }

  printf ("All integer PCM packing tests passed.\n");
  return 0;
}
