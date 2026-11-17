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

 function: unit tests for the overlap-add window and granule position
           contract of the Vorbis decoder.

           The PCM interval a legal stream is allowed to expose is
           bounded on both ends:
             * the front roll (preroll) discards the half-window of
               implicit samples the first long block's left edge would
               otherwise produce;
             * the end-of-stream granule position trims the padding in
               the final (possibly short) block.
           Everything in between is produced exactly once by the
           overlap-add machinery, including every long<->short window
           transition.  These tests encode signals that force both
           window sizes and prove, through two independent decoder
           paths, that:
             - the number of returned samples equals the number fed in,
               so overlap regions are neither emitted twice nor dropped;
             - the same duration is obtained whether the stream is read
               in one shot, through tiny read buffers, from a seekable
               source or streamed without seeking, or with the physical
               stream truncated right after the end-of-stream flag;
             - failures are reported with the documented negative
               status codes instead of silent truncation.

 ********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <ogg/ogg.h>
#include <vorbis/codec.h>
#include <vorbis/vorbisenc.h>
#include <vorbis/vorbisfile.h>

#ifndef M_PI
#  define M_PI (3.14159265358979323846)
#endif

/* A read length deliberately smaller than every encoded block so that
 * every page and every synthesis block is reached through several
 * callback invocations. */
#define TINY_READ       7
#define TEST_SRATE      44100
#define TEST_QUALITY    0.5f

/* ------------------------------------------------------------------ */
/* Test harness                                                        */
/* ------------------------------------------------------------------ */

static int check_count;
static int fail_count;

#define CHECK(cond, ...)                                               \
  do {                                                                 \
    check_count++;                                                     \
    if(!(cond)) {                                                      \
      fail_count++;                                                    \
      printf("    CHECK FAILED (line %d): ", __LINE__);               \
      printf(__VA_ARGS__);                                             \
      fputc('\n', stdout);                                             \
    }                                                                  \
  } while(0)

/* Human-readable rendering of every negative status the public APIs
 * may return.  The failure tests assert both the numeric code and the
 * presence of a description, so an undocumented value cannot slip past
 * silently. */
static const char *vorbis_status_str(int status)
{
  switch(status) {
  case OV_FALSE:      return "OV_FALSE: no data available yet";
  case OV_EOF:        return "OV_EOF: end of stream reached";
  case OV_HOLE:       return "OV_HOLE: gap or discontinuity in data";
  case OV_EREAD:      return "OV_EREAD: underlying read callback failed";
  case OV_EFAULT:     return "OV_EFAULT: internal consistency fault";
  case OV_EIMPL:      return "OV_EIMPL: feature not implemented";
  case OV_EINVAL:     return "OV_EINVAL: invalid argument or state";
  case OV_ENOTVORBIS: return "OV_ENOTVORBIS: data is not a Vorbis stream";
  case OV_EBADHEADER:return "OV_EBADHEADER: invalid Vorbis header";
  case OV_EVERSION:   return "OV_EVERSION: unsupported Vorbis version";
  case OV_ENOTAUDIO:  return "OV_ENOTAUDIO: packet is not an audio packet";
  case OV_EBADPACKET:return "OV_EBADPACKET: undecodable audio packet";
  case OV_EBADLINK:   return "OV_EBADLINK: corrupt logical bitstream link";
  case OV_ENOSEEK:    return "OV_ENOSEEK: stream is not seekable";
  default:            return NULL;
  }
}

/* ------------------------------------------------------------------ */
/* Growable byte buffer used as an in-memory Ogg transport             */
/* ------------------------------------------------------------------ */

typedef struct {
  unsigned char *data;
  size_t         len;
  size_t         cap;
} byte_buf;

static void byte_buf_init(byte_buf *buf)
{
  buf->data = NULL;
  buf->len = 0;
  buf->cap = 0;
}

static void byte_buf_clear(byte_buf *buf)
{
  free(buf->data);
  buf->data = NULL;
  buf->len = 0;
  buf->cap = 0;
}

static void byte_buf_append(byte_buf *buf, const void *src, size_t n)
{
  if(buf->len + n > buf->cap) {
    size_t cap = buf->cap ? buf->cap : 1024;
    while(cap < buf->len + n)
      cap *= 2;
    buf->data = realloc(buf->data, cap);
    if(!buf->data) {
      fprintf(stderr, "out of memory\n");
      exit(1);
    }
    buf->cap = cap;
  }
  memcpy(buf->data + buf->len, src, n);
  buf->len += n;
}

/* ------------------------------------------------------------------ */
/* Memory stream + ov_callbacks                                        */
/* ------------------------------------------------------------------ */

typedef struct {
  const unsigned char *data;
  size_t               len;
  size_t               pos;
  int                  seekable;
  int                  fail_after;  /* bytes until reads start failing */
  int                  fail_on_read;/* sticky: return an I/O error    */
} mem_stream;

static size_t mem_read(void *ptr, size_t size, size_t nmemb, void *arg)
{
  mem_stream *ms = arg;
  size_t want = size * nmemb;
  size_t avail;

  if(ms->fail_on_read) {
    errno = EIO;
    return 0;
  }
  if(ms->fail_after >= 0 && ms->pos >= (size_t)ms->fail_after) {
    ms->fail_on_read = 1;
    errno = EIO;
    return 0;
  }
  if(ms->pos >= ms->len)
    return 0;
  avail = ms->len - ms->pos;
  if(want > avail)
    want = avail;
  memcpy(ptr, ms->data + ms->pos, want);
  ms->pos += want;
  return want / size;
}

static int mem_seek(void *arg, ogg_int64_t offset, int whence)
{
  mem_stream *ms = arg;
  ogg_int64_t target;

  if(!ms->seekable)
    return -1;
  switch(whence) {
  case SEEK_SET: target = offset; break;
  case SEEK_CUR: target = (ogg_int64_t)ms->pos + offset; break;
  case SEEK_END: target = (ogg_int64_t)ms->len + offset; break;
  default:       return -1;
  }
  if(target < 0 || (size_t)target > ms->len)
    return -1;
  ms->pos = (size_t)target;
  return 0;
}

static int mem_close(void *arg)
{
  (void)arg;
  return 0;
}

static long mem_tell(void *arg)
{
  mem_stream *ms = arg;
  return ms->seekable ? (long)ms->pos : -1;
}

static const ov_callbacks seekable_callbacks = {
  mem_read, mem_seek, mem_close, mem_tell
};
static const ov_callbacks stream_callbacks = {
  mem_read, NULL, mem_close, NULL
};

/* ------------------------------------------------------------------ */
/* Encoding: build a complete Ogg/Vorbis stream in memory              */
/* ------------------------------------------------------------------ */

/* Signal with isolated wideband impulses on a quiet bed.  The psycho
 * acoustic model marks the transients, forcing short blocks around them,
 * while the uninterrupted quiet stretches stay in long blocks; a stream
 * long enough to hold several clean long blocks between the transients
 * therefore contains long->short, short->long and long->long
 * transitions (a transient placed mid-stream returns to long blocks
 * afterwards, one near the start covers the entry transition).  The
 * transients are deliberately sparse so quiet regions dominate. */
static float test_sample(long i, long total, int ch)
{
  float v;
  (void)total;

  if(i == 64 || i == 5000)
    v = 0.9f;
  else if(i == 65 || i == 5001)
    v = -0.75f;
  else
    v = 0.01f * sinf(2.0f * (float)M_PI * i / 37.0f + (float)ch);
  return v;
}

/* Streams shorter than this cannot contain several full long blocks
 * (2048 samples each at 44.1 kHz) plus surrounding transients, so only
 * streams at least this long are required to exercise both window
 * sizes. */
#define MIXED_WINDOW_MIN_SAMPLES 8192

static int encode_vorbis(const float *input, long samples, int channels,
                         byte_buf *out)
{
  ogg_stream_state os;
  ogg_page og;
  ogg_packet op;
  vorbis_info vi;
  vorbis_comment vc;
  vorbis_dsp_state vd;
  vorbis_block vb;
  int eos = 0;
  long done = 0;

  vorbis_info_init(&vi);
  if(vorbis_encode_init_vbr(&vi, channels, TEST_SRATE, TEST_QUALITY)) {
    fprintf(stderr, "vorbis_encode_init_vbr failed\n");
    return -1;
  }
  vorbis_comment_init(&vc);
  vorbis_comment_add_tag(&vc, "ENCODER", "test_window_granule");
  vorbis_analysis_init(&vd, &vi);
  vorbis_block_init(&vd, &vb);
  ogg_stream_init(&os, 0x564f5242);

  /* Three header packets, flushed onto their own pages. */
  {
    ogg_packet header, header_comm, header_code;
    vorbis_analysis_headerout(&vd, &vc, &header, &header_comm,
                              &header_code);
    ogg_stream_packetin(&os, &header);
    ogg_stream_packetin(&os, &header_comm);
    ogg_stream_packetin(&os, &header_code);
    while(ogg_stream_flush(&os, &og))
      byte_buf_append(out, og.header, og.header_len),
      byte_buf_append(out, og.body, og.body_len);
  }

  /* Feed the signal in chunks smaller than a long block so the encoder
   * actually gets to choose a window size per blockout.  After every
   * chunk, force an Ogg page flush so the physical stream contains many
   * pages with granule-position fenceposts; this makes the framer, the
   * tiny-read transport and the mid-stream truncation path realistic. */
  while(!eos) {
    int produced_packet = 0;

    if(done < samples) {
      enum { CHUNK = 192 };
      long count = samples - done;
      float **buffer;
      long i, c;
      if(count > CHUNK)
        count = CHUNK;
      buffer = vorbis_analysis_buffer(&vd, count);
      for(c = 0; c < channels; c++)
        for(i = 0; i < count; i++)
          buffer[c][i] = input[(done + i) * channels + c];
      vorbis_analysis_wrote(&vd, count);
      done += count;
    } else {
      vorbis_analysis_wrote(&vd, 0);
    }

    while(vorbis_analysis_blockout(&vd, &vb) == 1) {
      vorbis_analysis(&vb, NULL);
      vorbis_bitrate_addblock(&vb);
      while(vorbis_bitrate_flushpacket(&vd, &op)) {
        ogg_stream_packetin(&os, &op);
        produced_packet = 1;
        while(ogg_stream_pageout(&os, &og)) {
          byte_buf_append(out, og.header, og.header_len);
          byte_buf_append(out, og.body, og.body_len);
          if(ogg_page_eos(&og))
            eos = 1;
        }
      }
    }

    /* Bound the page span: flush whatever is pending unless this pass
     * already closed the stream with the EOS page. */
    if(!eos && produced_packet) {
      while(ogg_stream_flush(&os, &og)) {
        byte_buf_append(out, og.header, og.header_len);
        byte_buf_append(out, og.body, og.body_len);
        if(ogg_page_eos(&og))
          eos = 1;
      }
    }
  }

  ogg_stream_clear(&os);
  vorbis_block_clear(&vb);
  vorbis_dsp_clear(&vd);
  vorbis_comment_clear(&vc);
  vorbis_info_clear(&vi);
  return 0;
}

/* ------------------------------------------------------------------ */
/* Decoder path 1: raw libvorbis synthesis through ogg framing         */
/* ------------------------------------------------------------------ */

typedef struct {
  float *pcm;       /* interleaved */
  long   samples;   /* per-channel samples stored */
  long   cap;
  int    channels;
  /* window transition bookkeeping */
  int    saw_long, saw_short;
  int    transitions;   /* number of adjacent blocks changing W */
  int    long_to_short; /* long block directly followed by a short one */
  int    short_to_long; /* short block directly followed by a long one */
  long   last_W;
  int    have_W;
  int    eos_blocks;
  /* samples the final end-of-stream block still had to drain through
   * the overlap-add buffers after its packet was synthesized */
  long   drain_after_eos;
} raw_decode_result;

static void raw_result_init(raw_decode_result *r, int channels)
{
  r->cap = 4096;
  r->pcm = malloc(sizeof(float) * r->cap * channels);
  if(!r->pcm) {
    fprintf(stderr, "out of memory\n");
    exit(1);
  }
  r->samples = 0;
  r->channels = channels;
  r->saw_long = r->saw_short = 0;
  r->transitions = 0;
  r->long_to_short = r->short_to_long = 0;
  r->last_W = 0;
  r->have_W = 0;
  r->eos_blocks = 0;
  r->drain_after_eos = 0;
}

static void raw_result_clear(raw_decode_result *r)
{
  free(r->pcm);
  memset(r, 0, sizeof(*r));
}

static void raw_append(raw_decode_result *r, float **pcm, long n)
{
  long i, c;
  if(r->samples + n > r->cap) {
    while(r->samples + n > r->cap)
      r->cap *= 2;
    r->pcm = realloc(r->pcm, sizeof(float) * r->cap * r->channels);
    if(!r->pcm) {
      fprintf(stderr, "out of memory\n");
      exit(1);
    }
  }
  for(i = 0; i < n; i++)
    for(c = 0; c < r->channels; c++)
      r->pcm[(r->samples + i) * r->channels + c] = pcm[c][i];
  r->samples += n;
}

/* Returns 0 on success, otherwise the first negative libvorbis status
 * encountered. */
static int raw_decode(const unsigned char *data, size_t len,
                      int read_hint, raw_decode_result *r)
{
  ogg_sync_state oy;
  ogg_stream_state os;
  ogg_page og;
  ogg_packet op;
  vorbis_info vi;
  vorbis_comment vc;
  vorbis_dsp_state vd;
  vorbis_block vb;
  size_t pos = 0;
  int headers = 0;
  int stream_init = 0;
  int dsp_init = 0;
  int status = 0;
  int page_eos_seen = 0;
  int eos_done = 0;

  ogg_sync_init(&oy);
  vorbis_info_init(&vi);
  vorbis_comment_init(&vc);

  while(!eos_done) {
    /* Feed the framer in (possibly tiny) chunks. */
    while(1) {
      int got = ogg_sync_pageout(&oy, &og);
      if(got == 0)
        break;
      if(got < 0) {
        /* Lost page boundary; a legal stream must never hit this. */
        status = OV_HOLE;
        goto done;
      }
      if(!stream_init) {
        ogg_stream_init(&os, ogg_page_serialno(&og));
        stream_init = 1;
      }
      if(ogg_stream_pagein(&os, &og) < 0) {
        status = OV_EBADLINK;
        goto done;
      }
      if(ogg_page_eos(&og))
        page_eos_seen = 1;

      while(1) {
        int pr = ogg_stream_packetout(&os, &op);
        if(pr == 0)
          break;
        if(pr < 0) {
          status = OV_HOLE;
          goto done;
        }
        if(headers < 3) {
          int hr = vorbis_synthesis_headerin(&vi, &vc, &op);
          if(hr < 0) {
            status = hr;
            goto done;
          }
          headers++;
          if(headers == 3) {
            if(vi.channels != r->channels) {
              status = OV_EINVAL;
              goto done;
            }
            vorbis_synthesis_init(&vd, &vi);
            vorbis_block_init(&vd, &vb);
            dsp_init = 1;
          }
          continue;
        }

        {
          long samples_at_eos;
          int sr = vorbis_synthesis(&vb, &op);
          if(sr) {
            status = sr;
            goto done;
          }
          /* Record the window sizes actually coded in the stream. */
          if(r->have_W && vb.W != r->last_W) {
            r->transitions++;
            if(r->last_W && !vb.W)
              r->long_to_short = 1;
            else if(!r->last_W && vb.W)
              r->short_to_long = 1;
          }
          r->have_W = 1;
          r->last_W = vb.W;
          if(vb.W)
            r->saw_long = 1;
          else
            r->saw_short = 1;
          if(vb.eofflag)
            r->eos_blocks++;

          samples_at_eos = r->samples;
          vorbis_synthesis_blockin(&vd, &vb);
          while(1) {
            float **pcm;
            long n = vorbis_synthesis_pcmout(&vd, &pcm);
            if(n <= 0)
              break;
            raw_append(r, pcm, n);
            vorbis_synthesis_read(&vd, n);
          }
          /* Whatever the EOS block emitted at this point is the tail
           * drain: overlap-add samples that only became valid once the
           * final window was known, trimmed to the EOS granule. */
          if(vb.eofflag) {
            r->drain_after_eos = r->samples - samples_at_eos;
            /* The logical stream ends here; anything physically left in
             * the buffer is another stream or junk, not more PCM. */
            eos_done = 1;
          }
        }
      }
    }

    if(eos_done)
      break;
    if(pos >= len)
      break;
    {
      long want = read_hint;
      char *buf;
      if((size_t)want > len - pos)
        want = (long)(len - pos);
      buf = ogg_sync_buffer(&oy, want);
      memcpy(buf, data + pos, want);
      ogg_sync_wrote(&oy, want);
      pos += want;
    }
  }

  if(!page_eos_seen) {
    /* A truncated physical stream is allowed to end early, but a
     * complete legal stream must carry the end-of-stream flag. */
    status = OV_EOF;
    goto done;
  }
  if(headers < 3) {
    status = OV_EBADHEADER;
    goto done;
  }

done:
  if(dsp_init) {
    vorbis_block_clear(&vb);
    vorbis_dsp_clear(&vd);
  }
  if(stream_init)
    ogg_stream_clear(&os);
  vorbis_comment_clear(&vc);
  vorbis_info_clear(&vi);
  ogg_sync_clear(&oy);
  return status;
}

/* ------------------------------------------------------------------ */
/* Decoder path 2: vorbisfile high-level API                           */
/* ------------------------------------------------------------------ */

typedef struct {
  float *pcm;
  long   samples;
  long   cap;
  int    channels;
  ogg_int64_t final_offset;
  ogg_int64_t pcm_total;
  int    terminal_status; /* last non-positive ov_read_float status  */
} vf_decode_result;

static void vf_result_init(vf_decode_result *r, int channels)
{
  r->cap = 4096;
  r->pcm = malloc(sizeof(float) * r->cap * channels);
  if(!r->pcm) {
    fprintf(stderr, "out of memory\n");
    exit(1);
  }
  r->samples = 0;
  r->channels = channels;
  r->final_offset = -1;
  r->pcm_total = -1;
  r->terminal_status = 0;
}

static void vf_result_clear(vf_decode_result *r)
{
  free(r->pcm);
  memset(r, 0, sizeof(*r));
}

static int vf_append(vf_decode_result *r, float **pcm, long n)
{
  long i, c;
  if(r->samples + n > r->cap) {
    while(r->samples + n > r->cap)
      r->cap *= 2;
    r->pcm = realloc(r->pcm, sizeof(float) * r->cap * r->channels);
    if(!r->pcm)
      return -1;
  }
  for(i = 0; i < n; i++)
    for(c = 0; c < r->channels; c++)
      r->pcm[(r->samples + i) * r->channels + c] = pcm[c][i];
  r->samples += n;
  return 0;
}

/* Decode through ov_read_float with a bounded request size so the
 * overlap-add drain after EOS is exercised across multiple calls.
 * Returns the first negative status, or 0 when the stream ended
 * cleanly (ov_read_float returned 0). */
static int vf_decode(mem_stream *ms, const ov_callbacks *callbacks,
                     int request_size, vf_decode_result *r)
{
  OggVorbis_File vf;
  int open_status;
  int status = 0;

  open_status = ov_open_callbacks(ms, &vf, NULL, 0, *callbacks);
  if(open_status < 0)
    return open_status;

  r->channels = ov_info(&vf, -1)->channels;
  if(vf.seekable)
    r->pcm_total = ov_pcm_total(&vf, -1);

  while(1) {
    float **pcm = NULL;
    long got = ov_read_float(&vf, &pcm, request_size, NULL);
    if(got == 0) {
      r->terminal_status = OV_EOF;
      break;
    }
    if(got < 0) {
      r->terminal_status = (int)got;
      status = (int)got;
      break;
    }
    if(vf_append(r, pcm, got)) {
      status = OV_EFAULT;
      break;
    }
  }
  r->final_offset = ov_pcm_tell(&vf);
  ov_clear(&vf);
  return status;
}

/* ------------------------------------------------------------------ */
/* Individual checks                                                   */
/* ------------------------------------------------------------------ */

static float *make_signal(long samples, int channels)
{
  long i, c;
  float *sig = malloc(sizeof(float) * samples * channels);
  if(!sig) {
    fprintf(stderr, "out of memory\n");
    exit(1);
  }
  for(i = 0; i < samples; i++)
    for(c = 0; c < channels; c++)
      sig[i * channels + c] = test_sample(i, samples, c);
  return sig;
}

/* Core duration contract: both decoder paths must emit exactly as many
 * samples as were encoded, and vorbisfile's declared total and final
 * cursor must agree with that count. */
static void check_duration_contract(const byte_buf *ogg,
                                    const float *input, long samples,
                                    int channels, const char *label)
{
  raw_decode_result raw;
  vf_decode_result bulk, tiny;
  mem_stream ms_bulk, ms_tiny;
  int rc;

  printf("  [%s] duration contract (%ld samples, %d ch)\n",
         label, samples, channels);

  raw_result_init(&raw, channels);
  rc = raw_decode(ogg->data, ogg->len, TINY_READ, &raw);
  CHECK(rc == 0, "raw decode returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(raw.samples == samples,
        "raw path emitted %ld samples, expected %ld (front/tail granule "
        "trim mismatch)", raw.samples, samples);
  if(samples >= MIXED_WINDOW_MIN_SAMPLES) {
    CHECK(raw.saw_long && raw.saw_short,
          "stream did not exercise both window sizes (long=%d short=%d); "
          "window transitions are untested",
          raw.saw_long, raw.saw_short);
    CHECK(raw.transitions > 0,
          "no long<->short window transition observed (%d transitions)",
          raw.transitions);
    CHECK(raw.long_to_short && raw.short_to_long,
          "window switching was one-sided (long->short=%d, "
          "short->long=%d); overlap windows in both transition shapes "
          "are untested",
          raw.long_to_short, raw.short_to_long);
  } else {
    printf("    (window mixing skipped: stream shorter than %d samples)\n",
           MIXED_WINDOW_MIN_SAMPLES);
  }
  CHECK(raw.eos_blocks == 1,
        "expected exactly one end-of-stream block, got %d",
        raw.eos_blocks);
  /* The final block must leave no hidden samples behind: its drain plus
   * everything emitted earlier accounts for the whole stream. */
  CHECK(raw.drain_after_eos > 0 && raw.drain_after_eos <= samples,
        "EOS block drained %ld samples; expected a positive overlap tail "
        "bounded by the stream length", raw.drain_after_eos);
  /* Front roll: the first emitted sample must track the first encoded
   * one, proving no preroll/overlap samples leak in front. */
  {
    float expected0 = input[0];
    float got0 = raw.pcm[0];
    CHECK(fabsf(got0 - expected0) < 0.25f,
          "first decoded sample %f does not match first input %f; front "
          "roll leaked preroll samples", got0, expected0);
  }

  /* Seekable source, large requests. */
  memset(&ms_bulk, 0, sizeof(ms_bulk));
  ms_bulk.data = ogg->data;
  ms_bulk.len = ogg->len;
  ms_bulk.seekable = 1;
  ms_bulk.fail_after = -1;
  vf_result_init(&bulk, channels);
  rc = vf_decode(&ms_bulk, &seekable_callbacks, 4096, &bulk);
  CHECK(rc == 0, "seekable bulk decode returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(bulk.samples == samples,
        "seekable bulk decode emitted %ld samples, expected %ld",
        bulk.samples, samples);
  CHECK(bulk.pcm_total == samples,
        "ov_pcm_total reports %lld, expected %ld (EOS granule trim)",
        (long long)bulk.pcm_total, samples);
  CHECK(bulk.final_offset == samples,
        "ov_pcm_tell after drain is %lld, expected %ld (cursor must land "
        "on the trimmed end, not on padded block boundary)",
        (long long)bulk.final_offset, samples);
  /* The two independent decoder paths must agree sample-for-sample.  A
   * bug that drops or duplicates a block's overlap tail in one path
   * cannot hide when the other still honors the granule contract. */
  CHECK(bulk.samples == raw.samples,
        "vorbisfile emitted %ld samples but raw synthesis %ld; the paths "
        "disagree on the valid PCM interval", bulk.samples, raw.samples);
  if(bulk.samples == raw.samples)
    CHECK(memcmp(bulk.pcm, raw.pcm,
                 sizeof(float) * (size_t)samples * channels) == 0,
          "vorbisfile and raw synthesis PCM differ; overlap windows were "
          "stitched differently");
  raw_result_clear(&raw);

  /* Same bytes, tiny transport reads and tiny PCM requests.  The
   * overlap-add windows must still line up sample-for-sample, proving
   * no overlap region is duplicated or skipped. */
  memset(&ms_tiny, 0, sizeof(ms_tiny));
  ms_tiny.data = ogg->data;
  ms_tiny.len = ogg->len;
  ms_tiny.seekable = 1;
  ms_tiny.fail_after = -1;
  vf_result_init(&tiny, channels);
  rc = vf_decode(&ms_tiny, &seekable_callbacks, 3, &tiny);
  CHECK(rc == 0, "seekable tiny decode returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(tiny.samples == bulk.samples,
        "tiny reads emitted %ld samples, bulk reads %ld: chunking changed "
        "the effective duration", tiny.samples, bulk.samples);
  CHECK(tiny.final_offset == bulk.final_offset,
        "final pcm offset differs by read size (%lld vs %lld)",
        (long long)tiny.final_offset, (long long)bulk.final_offset);
  CHECK(memcmp(tiny.pcm, bulk.pcm,
               sizeof(float) * (size_t)samples * channels) == 0,
        "PCM differs between tiny and bulk reads; overlap windows were "
        "not stitched identically");
  vf_result_clear(&bulk);
  vf_result_clear(&tiny);
}

/* Streaming (unseekable) transport and junk appended after the EOS
 * page must obey the same per-sample duration contract.  A physical
 * stream that simply stops after end-of-stream is the normal drain
 * case; extra bytes that are not a new Ogg page must be ignored once
 * the final granule position has been honored. */
static void check_streaming_and_truncation(const byte_buf *ogg,
                                           long samples, int channels)
{
  byte_buf junked;
  static const unsigned char junk[32] = {
    0xff, 0xfe, 0x00, 0x01, 't', 'r', 'a', 'i', 'l', 'i', 'n', 'g',
    'b', 'y', 't', 'e', 's', '!', '!', '!', 0x7f, 0x80, 0xaa, 0x55,
    1, 2, 3, 4, 5, 6, 7, 8
  };
  vf_decode_result streamed, junk_tail;
  mem_stream ms_stream, ms_junk;
  raw_decode_result raw_junk;
  int rc;

  printf("  [stream] unseekable open (%ld samples)\n", samples);
  memset(&ms_stream, 0, sizeof(ms_stream));
  ms_stream.data = ogg->data;
  ms_stream.len = ogg->len;
  ms_stream.seekable = 0;
  ms_stream.fail_after = -1;
  vf_result_init(&streamed, channels);
  rc = vf_decode(&ms_stream, &stream_callbacks, 5, &streamed);
  CHECK(rc == 0, "streaming decode returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(streamed.samples == samples,
        "streaming path emitted %ld samples, expected %ld; preroll/EOS "
        "handling must be identical without seeking",
        streamed.samples, samples);
  CHECK(streamed.terminal_status == OV_EOF,
        "clean stream must terminate with OV_EOF semantics, got code %d",
        streamed.terminal_status);
  /* After the clean drain, one more read must report EOF rather than
   * replaying any overlap tail: the drained window is consumed once. */
  {
    mem_stream ms_more;
    OggVorbis_File vf;
    float **pcm = NULL;
    long first, second;
    memset(&ms_more, 0, sizeof(ms_more));
    ms_more.data = ogg->data;
    ms_more.len = ogg->len;
    ms_more.seekable = 0;
    ms_more.fail_after = -1;
    rc = ov_open_callbacks(&ms_more, &vf, NULL, 0, stream_callbacks);
    CHECK(rc == 0, "reopen for drain check failed (%d)", rc);
    if(rc == 0) {
      first = ov_read_float(&vf, &pcm, 1, NULL);
      second = ov_read_float(&vf, &pcm, 1, NULL);
      CHECK(first >= 0 && second >= 0,
            "drain check hit error %ld/%ld", first, second);
      /* Consume the whole stream one sample at a time. */
      while(1) {
        long g = ov_read_float(&vf, &pcm, 1, NULL);
        if(g <= 0) {
          CHECK(g == 0, "drain loop ended with status %ld, expected 0", g);
          break;
        }
      }
      CHECK(ov_pcm_tell(&vf) == samples,
            "one-sample-at-a-time drain ended at %lld, expected %ld",
            (long long)ov_pcm_tell(&vf), samples);
      /* A further read past the drained end must return 0 forever. */
      CHECK(ov_read_float(&vf, &pcm, 1, NULL) == 0,
            "read after full drain returned non-zero: overlap tail was "
            "replayed");
      ov_clear(&vf);
    }
  }
  vf_result_clear(&streamed);

  printf("  [junk  ] non-Ogg bytes appended after the EOS flag "
         "(%ld samples)\n", samples);
  byte_buf_init(&junked);
  byte_buf_append(&junked, ogg->data, ogg->len);
  byte_buf_append(&junked, junk, sizeof(junk));

  memset(&ms_junk, 0, sizeof(ms_junk));
  ms_junk.data = junked.data;
  ms_junk.len = junked.len;
  ms_junk.seekable = 1;
  ms_junk.fail_after = -1;
  vf_result_init(&junk_tail, channels);
  rc = vf_decode(&ms_junk, &seekable_callbacks, 11, &junk_tail);
  CHECK(rc == 0, "junk-tailed decode returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(junk_tail.samples == samples,
        "junk-tailed stream emitted %ld samples, expected %ld; the EOS "
        "granule position alone must define the valid tail",
        junk_tail.samples, samples);
  CHECK(junk_tail.final_offset == samples,
        "junk-tailed stream ended at offset %lld, expected %ld",
        (long long)junk_tail.final_offset, samples);
  vf_result_clear(&junk_tail);

  /* Same bytes through the raw path: drain must stop at the EOS granule
   * and the trailing junk must surface as an EOF, not extra PCM. */
  raw_result_init(&raw_junk, channels);
  rc = raw_decode(junked.data, junked.len, TINY_READ, &raw_junk);
  CHECK(rc == 0, "raw decode of junk-tailed stream returned %s",
        rc ? vorbis_status_str(rc) : "ok");
  CHECK(raw_junk.samples == samples,
        "raw junk-tailed decode emitted %ld samples, expected %ld",
        raw_junk.samples, samples);
  CHECK(raw_junk.drain_after_eos > 0,
        "EOS block produced no overlap drain; end-of-window samples "
        "would be missing");
  raw_result_clear(&raw_junk);
  byte_buf_clear(&junked);
}

/* A stream cut before its natural end loses the end granule; the raw
 * decoder must flag this rather than silently emit padded samples.
 * When several audio pages exist the cut is placed at an interior
 * granule fencepost (a strict partial decode); when all audio fits in
 * a single page, that page is truncated mid-body instead. */
static void check_missing_eos_is_reported(const byte_buf *ogg,
                                          long samples, int channels)
{
  /* Collect every Ogg page boundary with its granule position.  The
   * framer is fed one byte at a time so the byte offset at which each
   * completed page begins can be tracked externally (libogg's internal
   * storage moves as it grows, so pointer arithmetic against it is not
   * valid). */
  size_t boundary[64];
  ogg_int64_t granule[64];
  int npage = 0;
  int i;
  size_t cut = 0;
  int interior = 0;
  size_t first_audio = (size_t)-1;
  raw_decode_result raw;
  int rc;

  {
    ogg_sync_state scan;
    ogg_page og;
    size_t feed;
    ogg_sync_init(&scan);
    for(feed = 0; feed < ogg->len; feed++) {
      char *sb = ogg_sync_buffer(&scan, 1);
      *sb = (char)ogg->data[feed];
      ogg_sync_wrote(&scan, 1);
      while(npage < 64 && ogg_sync_pageout(&scan, &og) > 0) {
        long span = og.header_len + og.body_len;
        boundary[npage] = (feed + 1) - (size_t)span;
        granule[npage] = ogg_page_granulepos(&og);
        npage++;
      }
    }
    ogg_sync_clear(&scan);
  }
  CHECK(npage >= 1, "no Ogg pages found for truncation test");
  if(npage < 1)
    return;

  /* Locate the first audio page (positive granule) and, if present,
   * choose the latest interior fencepost that still precedes the end. */
  for(i = 0; i < npage; i++) {
    if(granule[i] > 0 && first_audio == (size_t)-1)
      first_audio = boundary[i];
    if(granule[i] > 0 && granule[i] < samples) {
      cut = boundary[i];
      interior = 1;
    }
  }

  if(!interior) {
    /* Single audio page: cut that page in half, mid-body, so the
     * framer can never assemble a complete audio page. */
    CHECK(first_audio != (size_t)-1 && first_audio < ogg->len,
          "could not locate an audio page for truncation");
    if(first_audio == (size_t)-1 || first_audio >= ogg->len)
      return;
    cut = first_audio + (ogg->len - first_audio) / 2;
  }

  printf("  [no-eos] stream cut %s (%zu of %zu bytes)\n",
         interior ? "at an interior granule fencepost"
                  : "mid-page through the only audio page",
         cut, ogg->len);
  raw_result_init(&raw, channels);
  rc = raw_decode(ogg->data, cut, TINY_READ, &raw);
  CHECK(rc == OV_EOF,
        "truncation before EOS reported code %d (%s); expected OV_EOF so "
        "the missing end granule is never mistaken for valid tail PCM",
        rc, vorbis_status_str(rc) ? vorbis_status_str(rc) : "unknown");

  if(interior) {
    /* Cutting at a granule fencepost yields a strict partial decode:
     * real tail samples are withheld because no EOS granule confirms
     * them. */
    CHECK(raw.samples > 0 && raw.samples < samples,
          "interior cut decoded %ld of %ld samples; expected a strict "
          "partial decode", raw.samples, samples);
  } else {
    /* The only audio page never completes, so no PCM can be produced. */
    CHECK(raw.samples == 0,
          "mid-page cut decoded %ld samples; expected none",
          raw.samples);
  }
  raw_result_clear(&raw);
}

/* Failure interpretability: every malformed input must produce one of
 * the documented negative status codes, each with a description. */
static void check_failures_are_explainable(const byte_buf *ogg)
{
  static const unsigned char junk[64] = {
    'T','h','i','s',' ','i','s',' ','n','o','t',' ','a','n',' ','O','g','g',
    ' ','s','t','r','e','a','m',' ','a','t',' ','a','l','l','.','.','.','.',
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25
  };
  mem_stream ms;
  OggVorbis_File vf;
  int rc;
  int i;

  printf("  [error] failure status codes stay documented\n");

  /* Every code used anywhere in these tests must be explainable. */
  {
    static const int codes[] = {
      OV_FALSE, OV_EOF, OV_HOLE, OV_EREAD, OV_EFAULT, OV_EIMPL,
      OV_EINVAL, OV_ENOTVORBIS, OV_EBADHEADER, OV_EVERSION,
      OV_ENOTAUDIO, OV_EBADPACKET, OV_EBADLINK, OV_ENOSEEK
    };
    for(i = 0; i < (int)(sizeof(codes) / sizeof(codes[0])); i++)
      CHECK(vorbis_status_str(codes[i]) != NULL,
            "status code %d has no explanation", codes[i]);
  }

  /* Non-Ogg bytes: must be rejected at open, never opened as audio. */
  memset(&ms, 0, sizeof(ms));
  ms.data = junk;
  ms.len = sizeof(junk);
  ms.seekable = 1;
  ms.fail_after = -1;
  rc = ov_open_callbacks(&ms, &vf, NULL, 0, seekable_callbacks);
  CHECK(rc < 0, "garbage input opened successfully (rc=%d)", rc);
  CHECK(rc == OV_ENOTVORBIS || rc == OV_EBADHEADER,
        "garbage input gave %d (%s); expected a framing/header class error",
        rc, vorbis_status_str(rc) ? vorbis_status_str(rc) : "unknown");
  CHECK(vorbis_status_str(rc) != NULL,
        "garbage input error %d has no human-readable explanation", rc);
  if(rc == 0)
    ov_clear(&vf);

  /* Empty stream: same class of failure, still explainable. */
  memset(&ms, 0, sizeof(ms));
  ms.data = junk;
  ms.len = 0;
  ms.seekable = 1;
  ms.fail_after = -1;
  rc = ov_open_callbacks(&ms, &vf, NULL, 0, seekable_callbacks);
  CHECK(rc < 0 && vorbis_status_str(rc) != NULL,
        "empty input produced undocumented result %d", rc);

  /* I/O failure while headers are still being read. */
  memset(&ms, 0, sizeof(ms));
  ms.data = ogg->data;
  ms.len = ogg->len;
  ms.seekable = 1;
  ms.fail_after = 4; /* fail inside the first header page */
  ms.fail_on_read = 0;
  rc = ov_open_callbacks(&ms, &vf, NULL, 0, seekable_callbacks);
  CHECK(rc == OV_EREAD || rc == OV_ENOTVORBIS || rc == OV_EBADHEADER,
        "mid-header read failure gave %d (%s); expected an I/O or header "
        "error, not silent success",
        rc, vorbis_status_str(rc) ? vorbis_status_str(rc) : "unknown");
  CHECK(rc < 0 && vorbis_status_str(rc) != NULL,
        "read-failure result %d is undocumented", rc);

  /* Invalid API arguments must be rejected with OV_EINVAL rather than
   * dereferencing garbage. */
  {
    OggVorbis_File unopened;
    char bytebuf[64];
    float **pcm = NULL;
    int section = -1;
    long got;
    memset(&unopened, 0, sizeof(unopened));
    got = ov_read_float(&unopened, &pcm, 16, &section);
    CHECK(got == OV_EINVAL,
          "read on an unopened handle returned %ld; expected OV_EINVAL",
          got);
    got = ov_read(&unopened, bytebuf, (int)sizeof(bytebuf), 0, 2, 1,
                  &section);
    CHECK(got == OV_EINVAL,
          "ov_read on an unopened handle returned %ld; expected OV_EINVAL",
          got);
    /* word size 0 is nonsensical even on a properly opened stream. */
    memset(&ms, 0, sizeof(ms));
    ms.data = ogg->data;
    ms.len = ogg->len;
    ms.seekable = 1;
    ms.fail_after = -1;
    rc = ov_open_callbacks(&ms, &vf, NULL, 0, seekable_callbacks);
    CHECK(rc == 0, "legal stream failed to open (rc=%d)", rc);
    if(rc == 0) {
      got = ov_read(&vf, bytebuf, (int)sizeof(bytebuf), 0, 0, 1,
                    &section);
      CHECK(got == OV_EINVAL,
            "ov_read with word size 0 returned %ld; expected OV_EINVAL",
            got);
      CHECK(vorbis_status_str(OV_EINVAL) != NULL,
            "OV_EINVAL has no explanation");
      ov_clear(&vf);
    }
  }
}

/* ------------------------------------------------------------------ */

int main(void)
{
  /* Several lengths: below one short block, around one long block, and
   * long enough to force repeated window switching.  None is a multiple
   * of the 64-sample block grid, exercising the tail granule trim. */
  static const long lengths[] = { 17, 200, 1000, 4096, 12000 };
  static const int channel_sets[] = { 1, 2 };
  size_t li, ci;

  printf("Vorbis overlap window / granule position unit tests\n\n");

  for(ci = 0; ci < sizeof(channel_sets) / sizeof(channel_sets[0]); ci++) {
    int channels = channel_sets[ci];
    for(li = 0; li < sizeof(lengths) / sizeof(lengths[0]); li++) {
      long samples = lengths[li];
      byte_buf ogg;
      float *signal;

      byte_buf_init(&ogg);
      signal = make_signal(samples, channels);
      if(encode_vorbis(signal, samples, channels, &ogg)) {
        fail_count++;
        printf("  encoding failed for %ld samples / %d ch\n",
               samples, channels);
        free(signal);
        byte_buf_clear(&ogg);
        continue;
      }
      printf("%ld samples, %d channel(s), %zu encoded bytes\n",
             samples, channels, ogg.len);

      check_duration_contract(&ogg, signal, samples, channels, "seek ");
      check_streaming_and_truncation(&ogg, samples, channels);
      check_missing_eos_is_reported(&ogg, samples, channels);
      check_failures_are_explainable(&ogg);

      free(signal);
      byte_buf_clear(&ogg);
      putchar('\n');
    }
  }

  printf("----\n%d checks, %d failure(s)\n", check_count, fail_count);
  return fail_count ? 1 : 0;
}
