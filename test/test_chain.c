/********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE OggVorbis SOFTWARE CODEC SOURCE CODE.   *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2015             *
 * by the Xiph.Org Foundation https://xiph.org/                     *
 *                                                                  *
 ********************************************************************

 function: chained logical bitstream tests for libvorbisfile

 ********************************************************************/

/* These tests pin down the cross-link contract of the vorbisfile API:

   - A physical bitstream may be a chain of logical bitstreams ("links")
     whose sample rates and channel counts differ.  ov_read() reports the
     link it is decoding while ov_pcm_total()/ov_time_total() report both
     per-link and whole-chain totals.
   - ov_pcm_seek()/ov_time_seek() address the *physical* (concatenated)
     sample/time axis, but the conversion from time to samples must use
     the sample rate of the link that time falls in; folding the whole
     chain through a single rate is wrong.
   - The chain's end ("finite tail") is an inclusive PCM boundary but an
     exclusive time boundary: ov_pcm_seek(total) is a valid, empty
     position, while ov_time_seek(total) is not a valid target.  Errors
     from the two entry points must not be confused with the OV_ENOSEEK
     result of a non-seekable (streaming, "unlocatable") input.

   The fixture is generated in memory (a three-link chain with distinct
   rates, channel counts and durations), so the test does not depend on
   files on disk or on the platform's stdio. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ogg/ogg.h>
#include <vorbis/codec.h>
#include <vorbis/vorbisenc.h>
#include <vorbis/vorbisfile.h>

#define CHAIN_LINKS 3

/* Three links, deliberately different in sample rate and channel count.
   Durations are long enough that each link spans several Ogg pages, so
   page-granularity seeking really has to walk across a link boundary. */
static const long chain_rates[CHAIN_LINKS]    = {44100, 22050, 48000};
static const int  chain_channels[CHAIN_LINKS] = {1, 2, 1};
static const long chain_samples[CHAIN_LINKS]  = {150000, 66150, 100000};
static const long chain_serials[CHAIN_LINKS]  = {0x11223344L, 0x22334455L,
                                                 0x33445566L};

/* Cumulative position of each link on the physical (chained) axes. */
static ogg_int64_t chain_pcm[CHAIN_LINKS + 1];
static double chain_time[CHAIN_LINKS + 1];

/* ------------------------------------------------------------------ */
/* Minimal in-memory data source                                      */
/* ------------------------------------------------------------------ */

typedef struct {
  unsigned char *data;
  size_t len;      /* bytes written into the stream */
  size_t alloc;    /* size of the allocation behind data */
  size_t pos;      /* read cursor for the callbacks */
} memstream;

static void ms_free(memstream *ms){
  free(ms->data);
  ms->data = NULL;
  ms->len = 0;
  ms->alloc = 0;
  ms->pos = 0;
}

static int ms_append(memstream *ms, const void *src, size_t n){
  if(ms->len + n > ms->alloc){
    size_t cap = ms->alloc ? ms->alloc : 8192;
    unsigned char *p;
    while(cap < ms->len + n) cap *= 2;
    p = (unsigned char *)realloc(ms->data, cap);
    if(!p) return -1;
    ms->data = p;
    ms->alloc = cap;
  }
  memcpy(ms->data + ms->len, src, n);
  ms->len += n;
  return 0;
}

static int ms_append_page(memstream *ms, const ogg_page *og){
  if(ms_append(ms, og->header, (size_t)og->header_len)) return -1;
  return ms_append(ms, og->body, (size_t)og->body_len);
}

static size_t ms_read(void *ptr, size_t size, size_t nmemb, void *datasource){
  memstream *ms = (memstream *)datasource;
  size_t want;
  size_t got;
  size_t left;

  if(!size) return 0;
  want = size * nmemb;
  left = ms->len - ms->pos;
  got = want < left ? want : left;
  got -= got % size; /* only whole elements */
  if(got){
    memcpy(ptr, ms->data + ms->pos, got);
    ms->pos += got;
  }
  return got / size;
}

static int ms_seek(void *datasource, ogg_int64_t offset, int whence){
  memstream *ms = (memstream *)datasource;
  ogg_int64_t target;
  if(whence == SEEK_SET) target = offset;
  else if(whence == SEEK_CUR) target = (ogg_int64_t)ms->pos + offset;
  else if(whence == SEEK_END) target = (ogg_int64_t)ms->len + offset;
  else return -1;
  if(target < 0 || target > (ogg_int64_t)ms->len) return -1;
  ms->pos = (size_t)target;
  return 0;
}

static int ms_close(void *datasource){
  (void)datasource; /* the memory is owned by the caller */
  return 0;
}

static long ms_tell(void *datasource){
  return (long)((memstream *)datasource)->pos;
}

static ov_callbacks seekable_callbacks(void){
  ov_callbacks cb;
  cb.read_func = ms_read;
  cb.seek_func = ms_seek;
  cb.close_func = ms_close;
  cb.tell_func = ms_tell;
  return cb;
}

static ov_callbacks stream_callbacks(void){
  /* No seek_func/tell_func: libvorbisfile must treat this as unseekable. */
  ov_callbacks cb;
  cb.read_func = ms_read;
  cb.seek_func = NULL;
  cb.close_func = NULL;
  cb.tell_func = NULL;
  return cb;
}

/* ------------------------------------------------------------------ */
/* Fixture generation                                                */
/* ------------------------------------------------------------------ */

#define CHAIN_PI 3.14159265358979323846

static int encode_link(memstream *ms, long serialno, long rate, int channels,
                       long samples, double hz){
  ogg_stream_state os;
  ogg_page og;
  ogg_packet op;
  vorbis_info vi;
  vorbis_comment vc;
  vorbis_dsp_state vd;
  vorbis_block vb;
  float *left = NULL;
  float *right = NULL;
  long k;
  long fed;
  int i;
  int eos = 0;
  int ret = 0;
  int stream_open = 0;
  int block_open = 0;
  int dsp_open = 0;
  int comment_open = 0;
  int info_open = 0;

  left = (float *)malloc((size_t)samples * sizeof(*left));
  right = (float *)malloc((size_t)samples * sizeof(*right));
  if(!left || !right){ ret = -1; goto done; }

  for(k = 0; k < samples; k++){
    double t = (double)k / (double)rate;
    left[k] = (float)(0.8 * sin(2.0 * CHAIN_PI * hz * t));
    right[k] = (float)(0.3 * sin(2.0 * CHAIN_PI * hz * t + 0.7));
  }

  vorbis_info_init(&vi);
  info_open = 1;
  if(vorbis_encode_init_vbr(&vi, (long)channels, rate, 0.35f)){
    ret = -1;
    goto done;
  }
  vorbis_comment_init(&vc);
  comment_open = 1;
  vorbis_comment_add_tag(&vc, "ENCODER", "test_chain");
  vorbis_analysis_init(&vd, &vi);
  dsp_open = 1;
  vorbis_block_init(&vd, &vb);
  block_open = 1;
  ogg_stream_init(&os, (int)serialno);
  stream_open = 1;

  {
    ogg_packet header;
    ogg_packet header_comm;
    ogg_packet header_code;
    vorbis_analysis_headerout(&vd, &vc, &header, &header_comm, &header_code);
    ogg_stream_packetin(&os, &header);
    ogg_stream_packetin(&os, &header_comm);
    ogg_stream_packetin(&os, &header_code);
    while(!eos){
      int result = ogg_stream_flush(&os, &og);
      if(result == 0) break;
      if(ms_append_page(ms, &og)){ ret = -1; goto done; }
    }
  }

  fed = 0;
  while(fed < samples){
    long chunk = samples - fed;
    float **buffer;
    if(chunk > 2048) chunk = 2048;
    buffer = vorbis_analysis_buffer(&vd, (int)chunk);
    for(i = 0; i < channels; i++)
      memcpy(buffer[i], (i == 0 ? left : right) + fed,
             (size_t)chunk * sizeof(float));
    vorbis_analysis_wrote(&vd, (int)chunk);
    fed += chunk;
    while(vorbis_analysis_blockout(&vd, &vb) == 1){
      vorbis_analysis(&vb, NULL);
      vorbis_bitrate_addblock(&vb);
      while(vorbis_bitrate_flushpacket(&vd, &op)){
        int result;
        ogg_stream_packetin(&os, &op);
        while(!eos){
          result = ogg_stream_pageout(&os, &og);
          if(result == 0) break;
          if(ms_append_page(ms, &og)){ ret = -1; goto done; }
          if(ogg_page_eos(&og)) eos = 1;
        }
      }
    }
  }

  vorbis_analysis_wrote(&vd, 0);
  while(vorbis_analysis_blockout(&vd, &vb) == 1){
    vorbis_analysis(&vb, NULL);
    vorbis_bitrate_addblock(&vb);
    while(vorbis_bitrate_flushpacket(&vd, &op)){
      int result;
      ogg_stream_packetin(&os, &op);
      while(!eos){
        result = ogg_stream_pageout(&os, &og);
        if(result == 0) break;
        if(ms_append_page(ms, &og)){ ret = -1; goto done; }
        if(ogg_page_eos(&og)) eos = 1;
      }
    }
  }

  if(!eos) ret = -1; /* the link never got an end-of-stream page */

 done:
  if(stream_open) ogg_stream_clear(&os);
  if(block_open) vorbis_block_clear(&vb);
  if(dsp_open) vorbis_dsp_clear(&vd);
  if(comment_open) vorbis_comment_clear(&vc);
  if(info_open) vorbis_info_clear(&vi);
  free(left);
  free(right);
  return ret;
}

static int build_chain(memstream *ms){
  int i;
  memset(ms, 0, sizeof(*ms));
  for(i = 0; i < CHAIN_LINKS; i++){
    if(encode_link(ms, chain_serials[i], chain_rates[i], chain_channels[i],
                   chain_samples[i], 300.0 + 170.0 * i))
      return -1;
  }
  if(ms->len == 0) return -1;
  return 0;
}

/* Independent view of the encoded fixture: the chain must really contain
   CHAIN_LINKS BOS pages, one EOS page per link, and the EOS granule
   positions must match the sample counts we fed the encoder.  Anchoring
   the expectations in the stream itself keeps the seek assertions honest
   even if the encoder's page layout changes. */
static int inspect_chain(const memstream *src, long *eos_granulepos){
  memstream ms = *src;
  ogg_sync_state oy;
  ogg_page og;
  int bos = 0;
  int eos = 0;
  int result = 0;

  ms.pos = 0;
  ogg_sync_init(&oy);
  while(1){
    char *buffer = ogg_sync_buffer(&oy, 4096);
    size_t room = 4096;
    size_t got;
    int page_result;
    if(ms.pos + room > ms.len) room = ms.len - ms.pos;
    if(room == 0) break;
    got = ms_read(buffer, 1, room, &ms);
    if(got == 0) break;
    ogg_sync_wrote(&oy, (long)got);
    while((page_result = ogg_sync_pageout(&oy, &og)) == 1){
      int i;
      if(ogg_page_bos(&og)){
        for(i = 0; i < CHAIN_LINKS; i++)
          if(chain_serials[i] == ogg_page_serialno(&og)) break;
        if(i == CHAIN_LINKS){ result = -1; goto done; }
        if(i != bos){ result = -1; goto done; }
        bos++;
      }
      if(ogg_page_eos(&og)){
        int i;
        for(i = 0; i < CHAIN_LINKS; i++)
          if(chain_serials[i] == ogg_page_serialno(&og)) break;
        if(i == CHAIN_LINKS || eos_granulepos[i] != -1){ result = -1; goto done; }
        eos_granulepos[i] = (long)ogg_page_granulepos(&og);
        eos++;
      }
    }
  }
  if(bos != CHAIN_LINKS || eos != CHAIN_LINKS) result = -1;

 done:
  ogg_sync_clear(&oy);
  return result;
}

/* ------------------------------------------------------------------ */
/* Check helpers                                                      */
/* ------------------------------------------------------------------ */

static int checks_run;
static int checks_failed;

static void check_true(int ok, const char *expr, const char *file, int line){
  checks_run++;
  if(!ok){
    checks_failed++;
    fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
  }
}

static void check_i64(ogg_int64_t got, ogg_int64_t want,
                      const char *expr, const char *file, int line){
  checks_run++;
  if(got != want){
    checks_failed++;
    fprintf(stderr, "FAIL %s:%d: %s: got %ld, want %ld\n",
            file, line, expr, (long)got, (long)want);
  }
}

static void check_near(double got, double want, double tol,
                       const char *expr, const char *file, int line){
  checks_run++;
  if(!(fabs(got - want) <= tol)){
    checks_failed++;
    fprintf(stderr, "FAIL %s:%d: %s: got %.17g, want %.17g (tol %.3g)\n",
            file, line, expr, got, want, tol);
  }
}

#define CHECK(cond)       check_true(!!(cond), #cond, __FILE__, __LINE__)
#define CHECK_I64(g, w)   check_i64((ogg_int64_t)(g), (ogg_int64_t)(w), \
                                    #g, __FILE__, __LINE__)
#define CHECK_NEAR(g, w, t) check_near((g), (w), (t), #g, __FILE__, __LINE__)

static ogg_int64_t i64_abs(ogg_int64_t v){
  return v < 0 ? -v : v;
}

/* Verify that the decoder reports the link we expect for the current
   position.  This is the "returned stream number" half of the contract:
   both ov_info(-1) and ov_serialnumber(-1) have to agree with the link
   that owns the sample position. */
static void check_current_link(OggVorbis_File *vf, int link,
                               const char *file, int line){
  vorbis_info *vi = ov_info(vf, -1);
  check_true(vi != NULL, "ov_info(vf,-1) != NULL", file, line);
  if(vi){
    check_i64(vi->rate, chain_rates[link], "ov_info(-1)->rate", file, line);
    check_i64(vi->channels, chain_channels[link],
              "ov_info(-1)->channels", file, line);
  }
  check_i64(ov_serialnumber(vf, -1), chain_serials[link],
            "ov_serialnumber(-1)", file, line);
}

#define CHECK_LINK(vf, link) check_current_link((vf), (link), __FILE__, __LINE__)

static long read_pcm(OggVorbis_File *vf, char *buffer, int size, int *bitstream){
  return ov_read(vf, buffer, size, 0, 2, 1, bitstream);
}

/* ------------------------------------------------------------------ */
/* Tests on a seekable chain                                          */
/* ------------------------------------------------------------------ */

static void test_structure(OggVorbis_File *vf){
  ogg_int64_t pcm_total = 0;
  ogg_int64_t raw_total = 0;
  ogg_int64_t raw_sum = 0;
  double time_sum = 0.0;
  int i;

  printf("  structure and totals\n");

  CHECK_I64(ov_seekable(vf), 1);
  CHECK_I64(ov_streams(vf), CHAIN_LINKS);

  for(i = 0; i < CHAIN_LINKS; i++){
    vorbis_info *vi = ov_info(vf, i);
    CHECK(vi != NULL);
    if(vi){
      CHECK_I64(vi->rate, chain_rates[i]);
      CHECK_I64(vi->channels, chain_channels[i]);
    }
    CHECK_I64(ov_serialnumber(vf, i), chain_serials[i]);
    /* Per-link statistics must be the raw sample counts of that link,
       independent of the surrounding links. */
    CHECK_I64(ov_pcm_total(vf, i), chain_samples[i]);
    CHECK_NEAR(ov_time_total(vf, i),
               (double)chain_samples[i] / (double)chain_rates[i], 1e-12);
    CHECK(ov_raw_total(vf, i) > 0);

    pcm_total += ov_pcm_total(vf, i);
    raw_sum += ov_raw_total(vf, i);
    time_sum += ov_time_total(vf, i);
  }

  /* Link indices outside [0,links) are not usable: they are reported as
     range errors rather than silently clamped. */
  CHECK(ov_info(vf, CHAIN_LINKS) == NULL);
  CHECK_I64(ov_pcm_total(vf, CHAIN_LINKS), OV_EINVAL);
  CHECK_I64(ov_raw_total(vf, CHAIN_LINKS), OV_EINVAL);
  CHECK(ov_time_total(vf, CHAIN_LINKS) == (double)OV_EINVAL);

  /* The whole-chain totals are exactly the sums of the per-link totals;
     a single rate cannot describe the chain. */
  CHECK_I64(ov_pcm_total(vf, -1), pcm_total);
  CHECK_NEAR(ov_time_total(vf, -1), time_sum, 1e-12);
  raw_total = ov_raw_total(vf, -1);
  CHECK_I64(raw_total, raw_sum);

  /* Guard the usefulness of the rest of the test: at least one link must
     use a rate other than link 0's, and for every such link the duration
     it contributes must differ measurably from what folding it through
     link 0's rate would give.  Without this the single-rate bug the tests
     target could hide behind an accidentally compatible fixture. */
  {
    int divergent = 0;
    CHECK(ov_time_total(vf, -1) > 0.0);
    for(i = 1; i < CHAIN_LINKS; i++){
      double own = (double)chain_samples[i] / (double)chain_rates[i];
      double folded = (double)chain_samples[i] / (double)chain_rates[0];
      if(chain_rates[i] != chain_rates[0]){
        CHECK(fabs(own - folded) > 1e-6);
        divergent++;
      }
    }
    CHECK(divergent > 0);
    CHECK(ov_time_total(vf, -1) != (double)pcm_total / (double)chain_rates[0]);
  }
}

static void test_pcm_boundaries(OggVorbis_File *vf){
  char buffer[8192];
  int bitstream;
  long got;
  int i;

  printf("  pcm seek across link boundaries\n");

  for(i = 0; i < CHAIN_LINKS; i++){
    ogg_int64_t boundary = chain_pcm[i];
    int link = i;

    /* Exactly on the boundary: the new link's first sample is next. */
    CHECK_I64(ov_pcm_seek(vf, boundary), 0);
    CHECK_I64(ov_pcm_tell(vf), boundary);
    CHECK_LINK(vf, link);

    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    CHECK(got > 0);
    CHECK_I64(bitstream, link);
    CHECK_I64(got % (2 * chain_channels[link]), 0);
    CHECK_I64(ov_pcm_tell(vf),
              boundary + got / (2 * chain_channels[link]));
    CHECK_LINK(vf, link);
  }

  /* One sample before each interior boundary: exactly the last sample of
     the previous link, then the boundary must hand over to the next link
     without losing or duplicating a sample. */
  for(i = 1; i < CHAIN_LINKS; i++){
    ogg_int64_t boundary = chain_pcm[i];
    int prev = i - 1;

    CHECK_I64(ov_pcm_seek(vf, boundary - 1), 0);
    CHECK_I64(ov_pcm_tell(vf), boundary - 1);
    CHECK_LINK(vf, prev);

    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    CHECK_I64(got, 2 * chain_channels[prev]); /* the final sample, one frame */
    CHECK_I64(bitstream, prev);
    CHECK_I64(ov_pcm_tell(vf), boundary);

    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    CHECK(got > 0);
    CHECK_I64(bitstream, i);
    CHECK_I64(got % (2 * chain_channels[i]), 0);
    CHECK_I64(ov_pcm_tell(vf),
              boundary + got / (2 * chain_channels[i]));
    CHECK_LINK(vf, i);
  }

  /* One sample after each interior boundary. */
  for(i = 1; i < CHAIN_LINKS; i++){
    ogg_int64_t boundary = chain_pcm[i];
    int link = i;

    CHECK_I64(ov_pcm_seek(vf, boundary + 1), 0);
    CHECK_I64(ov_pcm_tell(vf), boundary + 1);
    CHECK_LINK(vf, link);

    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    CHECK(got > 0);
    CHECK_I64(bitstream, link);
    CHECK_I64(got % (2 * chain_channels[link]), 0);
    CHECK_I64(ov_pcm_tell(vf),
              boundary + 1 + got / (2 * chain_channels[link]));
  }
}

static void test_time_boundaries(OggVorbis_File *vf){
  char buffer[8192];
  int bitstream;
  long got;
  int i;

  printf("  time seek across link boundaries\n");

  /* The cumulative start time of each link must land exactly on that
     link's first sample, with that link's rate/channels/serialno. */
  for(i = 0; i < CHAIN_LINKS; i++){
    CHECK_I64(ov_time_seek(vf, chain_time[i]), 0);
    CHECK_I64(ov_pcm_tell(vf), chain_pcm[i]);
    CHECK_LINK(vf, i);

    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    CHECK(got > 0);
    CHECK_I64(bitstream, i);
  }

  /* Interior positions are addressed in whole-chain time: the offset in
     seconds is relative to the link's *own* start, and the sample offset
     must be derived with that link's rate.  A one-sample tolerance
     absorbs floating-point rounding in the time->sample conversion; a
     wrong single-rate conversion would be thousands of samples away. */
  for(i = 0; i < CHAIN_LINKS; i++){
    long offsets[3];
    int j;
    offsets[0] = 1;
    offsets[1] = chain_samples[i] / 2;
    offsets[2] = chain_samples[i] - 1;

    for(j = 0; j < 3; j++){
      double seconds = chain_time[i] +
                       (double)offsets[j] / (double)chain_rates[i];
      ogg_int64_t expect = chain_pcm[i] + offsets[j];
      ogg_int64_t naive;
      ogg_int64_t pos;

      CHECK_I64(ov_time_seek(vf, seconds), 0);
      pos = ov_pcm_tell(vf);
      CHECK(i64_abs(pos - expect) <= 1);
      CHECK_LINK(vf, i);

      /* What a single-rate (link 0) conversion would have produced.  Only
         a mid-link offset is a meaningful probe: at a link boundary the
         single-rate result can coincide with the true position, and the
         drift is proportional to (seconds into the chain) times the rate
         difference.  Demand at least half of the drift that the link's own
         elapsed time alone accounts for. */
      if(i > 0 && j == 1){
        ogg_int64_t lower =
          (ogg_int64_t)((double)offsets[j] / (double)chain_rates[i] *
                        fabs((double)chain_rates[0] -
                             (double)chain_rates[i]) * 0.5);
        naive = (ogg_int64_t)(seconds * (double)chain_rates[0]);
        CHECK(lower > 0);
        CHECK(i64_abs(naive - pos) > lower);
      }

      bitstream = -1;
      got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
      CHECK(got > 0);
      CHECK_I64(bitstream, i);
      CHECK_I64(ov_pcm_tell(vf),
                pos + got / (2 * chain_channels[i]));
    }
  }

  /* ov_time_tell() after a pcm seek has to express the same
     cumulative-time accounting. */
  for(i = 0; i < CHAIN_LINKS; i++){
    long offset = chain_samples[i] / 3;
    double expect = chain_time[i] + (double)offset / (double)chain_rates[i];
    CHECK_I64(ov_pcm_seek(vf, chain_pcm[i] + offset), 0);
    CHECK_NEAR(ov_time_tell(vf), expect, 1e-9);
  }
}

static void test_page_seeks(OggVorbis_File *vf){
  char buffer[8192];
  int bitstream;
  long got;
  ogg_int64_t pos;
  ogg_int64_t total = chain_pcm[CHAIN_LINKS];
  int i;

  printf("  page-granularity seeks\n");

  /* At every link boundary the page seek must land on the closest page
     at or before the target, i.e. never past it, and on the correct
     link; reading from there must reach the target without errors. */
  for(i = 0; i < CHAIN_LINKS; i++){
    ogg_int64_t target = chain_pcm[i];
    int link = i;

    CHECK_I64(ov_pcm_seek_page(vf, target), 0);
    pos = ov_pcm_tell(vf);
    CHECK(pos <= target);
    CHECK_LINK(vf, link);

    while(ov_pcm_tell(vf) < target){
      bitstream = -1;
      got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
      CHECK(got > 0);
      if(got <= 0) break;
      CHECK_I64(bitstream, link);
    }
    CHECK(ov_pcm_tell(vf) >= target);
  }

  /* Page seek at the finite tail: success is a position at or before the
     end, and draining from it must produce exactly the remaining
     samples of the last link. */
  CHECK_I64(ov_pcm_seek_page(vf, total), 0);
  pos = ov_pcm_tell(vf);
  CHECK(pos <= total);
  CHECK(pos >= chain_pcm[CHAIN_LINKS - 1]);
  CHECK_LINK(vf, CHAIN_LINKS - 1);
  {
    long drained = 0;
    while(1){
      bitstream = -1;
      got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
      if(got == 0) break;
      CHECK(got > 0);
      if(got <= 0) break;
      CHECK_I64(bitstream, CHAIN_LINKS - 1);
      CHECK_I64(got % (2 * chain_channels[CHAIN_LINKS - 1]), 0);
      drained += got / (2 * chain_channels[CHAIN_LINKS - 1]);
    }
    CHECK_I64(drained, total - pos);
    CHECK_I64(ov_pcm_tell(vf), total);
  }
}

static void test_finite_tail(OggVorbis_File *vf){
  char buffer[8192];
  int bitstream;
  long got;
  ogg_int64_t total = chain_pcm[CHAIN_LINKS];
  ogg_int64_t kept;
  int last = CHAIN_LINKS - 1;

  printf("  finite tail\n");

  /* The end of the chain is a valid, empty PCM position. */
  CHECK_I64(ov_pcm_seek(vf, total), 0);
  CHECK_I64(ov_pcm_tell(vf), total);
  CHECK_LINK(vf, last);
  CHECK_NEAR(ov_time_tell(vf), chain_time[CHAIN_LINKS], 1e-9);

  bitstream = -1;
  got = read_pcm(vf, buffer, sizeof(buffer), &bitstream);
  CHECK_I64(got, 0); /* EOF, not an error and not a bogus sample */
  got = read_pcm(vf, buffer, sizeof(buffer), &bitstream);
  CHECK_I64(got, 0); /* EOF is stable across calls */
  CHECK_I64(ov_pcm_tell(vf), total);

  /* The last sample of the chain is readable exactly once. */
  CHECK_I64(ov_pcm_seek(vf, total - 1), 0);
  CHECK_I64(ov_pcm_tell(vf), total - 1);
  CHECK_LINK(vf, last);
  bitstream = -1;
  got = read_pcm(vf, buffer, sizeof(buffer), &bitstream);
  CHECK_I64(got, 2 * chain_channels[last]);
  CHECK_I64(bitstream, last);
  CHECK_I64(ov_pcm_tell(vf), total);
  got = read_pcm(vf, buffer, sizeof(buffer), &bitstream);
  CHECK_I64(got, 0);

  /* The end time is exclusive: the whole-chain duration itself is not a
     locatable time, but the last sample is.  This distinguishes "at the
     end of a locatable stream" from "not locatable at all" by both the
     return code and the position. */
  CHECK_I64(ov_time_seek(vf, chain_time[CHAIN_LINKS]), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), total); /* failed seek leaves the position */
  CHECK_I64(ov_time_seek_page(vf, chain_time[CHAIN_LINKS]), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), total);
  CHECK_I64(ov_time_seek(vf, chain_time[CHAIN_LINKS] -
                            0.5 / (double)chain_rates[last]), 0);
  CHECK_I64(ov_pcm_tell(vf), total - 1);
  CHECK_LINK(vf, last);
  bitstream = -1;
  got = read_pcm(vf, buffer, sizeof(buffer), &bitstream);
  CHECK_I64(got, 2 * chain_channels[last]);
  CHECK_I64(ov_pcm_tell(vf), total);

  /* Out-of-range PCM positions are rejected with OV_EINVAL on both the
     sample and the page entry points, and do not move the cursor. */
  CHECK_I64(ov_pcm_seek(vf, chain_pcm[1] + 1000), 0);
  kept = ov_pcm_tell(vf);
  CHECK_I64(ov_pcm_seek(vf, -1), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  CHECK_I64(ov_pcm_seek(vf, total + 1), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  CHECK_I64(ov_pcm_seek_page(vf, -1), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  CHECK_I64(ov_pcm_seek_page(vf, total + 1), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  CHECK_I64(ov_time_seek(vf, -1.0), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  CHECK_I64(ov_time_seek_page(vf, -1.0), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
  /* One second past the end of the chain is out of range as well. */
  CHECK_I64(ov_time_seek(vf, chain_time[CHAIN_LINKS] + 1.0), OV_EINVAL);
  CHECK_I64(ov_pcm_tell(vf), kept);
}

static void test_seekable_sequential(OggVorbis_File *vf){
  char buffer[8192];
  int bitstream;
  long got;
  long per_link[CHAIN_LINKS];
  int seen_links = 0;
  int last = -1;
  ogg_int64_t last_pcm = 0;
  int i;

  printf("  sequential decode of the seekable chain\n");

  memset(per_link, 0, sizeof(per_link));
  CHECK_I64(ov_pcm_seek(vf, 0), 0);

  while(1){
    bitstream = -1;
    got = read_pcm(vf, buffer, (int)sizeof(buffer), &bitstream);
    if(got == 0) break;
    CHECK(got > 0);
    if(got <= 0) break;
    CHECK(bitstream >= 0 && bitstream < CHAIN_LINKS);
    if(!(bitstream >= 0 && bitstream < CHAIN_LINKS)) break;
    CHECK_I64(got % (2 * chain_channels[bitstream]), 0);
    if(bitstream != last){
      CHECK_I64(bitstream, last + 1); /* links appear once, in order */
      seen_links++;
      last = bitstream;
      CHECK_LINK(vf, bitstream);
    }
    CHECK(ov_pcm_tell(vf) >= last_pcm);
    last_pcm = ov_pcm_tell(vf);
    per_link[bitstream] += got / (2 * chain_channels[bitstream]);
  }

  CHECK_I64(seen_links, CHAIN_LINKS);
  for(i = 0; i < CHAIN_LINKS; i++)
    CHECK_I64(per_link[i], chain_samples[i]);
  CHECK_I64(ov_pcm_tell(vf), chain_pcm[CHAIN_LINKS]);
  CHECK_NEAR(ov_time_tell(vf), chain_time[CHAIN_LINKS], 1e-9);
}

/* ------------------------------------------------------------------ */
/* Tests on a non-seekable ("unlocatable") chain                      */
/* ------------------------------------------------------------------ */

static void test_unseekable(const memstream *src){
  OggVorbis_File vf;
  memstream ms;
  ov_callbacks cb;
  char buffer[8192];
  int bitstream;
  long got;
  long per_link[CHAIN_LINKS];
  int seen_links = 0;
  int last_link = -1;
  int holes = 0;
  int fatal = 0;

  double derived_time = 0.0;
  int i;

  printf("  non-seekable chain\n");

  ms = *src;
  ms.pos = 0;
  cb = stream_callbacks();
  CHECK_I64(ov_open_callbacks(&ms, &vf, NULL, 0, cb), 0);

  /* A non-seekable input cannot expose the chain structure or its
     totals: there is exactly one "logical bitstream" visible at a time
     and ov_streams() reports 1 by contract. */
  CHECK_I64(ov_seekable(&vf), 0);
  CHECK_I64(ov_streams(&vf), 1);
  CHECK_I64(ov_pcm_total(&vf, -1), OV_EINVAL);
  CHECK_I64(ov_pcm_total(&vf, 0), OV_EINVAL);
  CHECK_I64(ov_raw_total(&vf, -1), OV_EINVAL);
  CHECK(ov_time_total(&vf, -1) == (double)OV_EINVAL);

  /* Every seeking entry point must say "unseekable" (OV_ENOSEEK), not
     pretend success and not report a range error. */
  CHECK_I64(ov_pcm_seek(&vf, 0), OV_ENOSEEK);
  CHECK_I64(ov_pcm_seek_page(&vf, 0), OV_ENOSEEK);
  CHECK_I64(ov_time_seek(&vf, 0.0), OV_ENOSEEK);
  CHECK_I64(ov_time_seek_page(&vf, 0.0), OV_ENOSEEK);
  CHECK_I64(ov_raw_seek(&vf, 0), OV_ENOSEEK);

  /* The failed seeks must not have disturbed the sequential decoder. */
  CHECK_I64(ov_pcm_tell(&vf), 0);
  CHECK_LINK(&vf, 0);

  memset(per_link, 0, sizeof(per_link));
  while(1){
    bitstream = -1;
    got = read_pcm(&vf, buffer, (int)sizeof(buffer), &bitstream);
    if(got == 0) break;
    if(got < 0){
      /* The only tolerated negative result is a hole at a packet that
         spans a link boundary; any other error is a real failure. */
      if(got == OV_HOLE){
        /* A hole is exactly the handover point: the link we were in has
           already delivered every one of its samples.  The link-identity
           accessors are deliberately *not* probed here: libvorbisfile
           has already parsed the next link's headers (ov_info) while
           ov_serialnumber() still names the outgoing link, so there is no
           single well-defined "current link" at the discontinuity itself.
           The reported link becomes unambiguous again on the next read,
           which the switch check below covers. */
        holes++;
        CHECK(last_link >= 0 && last_link < CHAIN_LINKS);
        if(last_link >= 0 && last_link < CHAIN_LINKS)
          CHECK_I64(per_link[last_link], chain_samples[last_link]);
      }else
        fatal++;
      continue;
    }
    CHECK(bitstream >= 0 && bitstream < CHAIN_LINKS);
    if(!(bitstream >= 0 && bitstream < CHAIN_LINKS)) break;
    CHECK_I64(got % (2 * chain_channels[bitstream]), 0);
    if(bitstream != last_link){
      /* ov_read() increments the reported link as the decoder crosses
         each boundary, even though ov_streams() stays at 1. */
      CHECK_I64(bitstream, last_link + 1);
      seen_links++;
      last_link = bitstream;
      CHECK_LINK(&vf, bitstream);
    }
    per_link[bitstream] += got / (2 * chain_channels[bitstream]);
    CHECK(per_link[bitstream] <= chain_samples[bitstream]);
  }

  CHECK_I64(fatal, 0);
  CHECK_I64(seen_links, CHAIN_LINKS);
  /* One hole per interior boundary: the end of a link is reported as a
     discontinuity in the *decode* stream, not as end-of-stream. */
  CHECK_I64(holes, CHAIN_LINKS - 1);
  for(i = 0; i < CHAIN_LINKS; i++){
    CHECK_I64(per_link[i], chain_samples[i]);
    derived_time += (double)per_link[i] / (double)chain_rates[i];
  }
  /* The per-link sample counts reproduce the chained duration that a
     seekable stream would report, while ov_pcm_total()/ov_time_total()
     above stayed unavailable rather than inventing a single-rate value. */
  CHECK_NEAR(derived_time, chain_time[CHAIN_LINKS], 1e-12);
  /* ov_time_tell() on an unseekable stream can only describe the current
     link's rate, so it must not be mistaken for the chained duration. */
  CHECK(chain_time[CHAIN_LINKS] > 0.0);
  CHECK(fabs(ov_time_tell(&vf) - chain_time[CHAIN_LINKS]) >
        0.05 * chain_time[CHAIN_LINKS]);
  CHECK_NEAR(ov_time_tell(&vf),
             (double)ov_pcm_tell(&vf) / (double)chain_rates[CHAIN_LINKS - 1],
             1e-9);

  /* Even at end-of-stream the seeking entry points keep reporting
     OV_ENOSEEK; a streaming chain never becomes locatable. */
  CHECK_I64(ov_pcm_seek(&vf, chain_pcm[CHAIN_LINKS]), OV_ENOSEEK);
  CHECK_I64(ov_time_seek(&vf, chain_time[CHAIN_LINKS]), OV_ENOSEEK);

  CHECK_I64(ov_clear(&vf), 0);
}

/* ------------------------------------------------------------------ */

int main(void){
  memstream ms;
  long eos_granulepos[CHAIN_LINKS];
  OggVorbis_File vf;
  ov_callbacks cb;
  int i;

  chain_pcm[0] = 0;
  chain_time[0] = 0.0;
  for(i = 0; i < CHAIN_LINKS; i++){
    chain_pcm[i + 1] = chain_pcm[i] + chain_samples[i];
    chain_time[i + 1] = chain_time[i] +
                        (double)chain_samples[i] / (double)chain_rates[i];
  }

  CHECK_I64(build_chain(&ms), 0);
  if(checks_failed){
    fprintf(stderr, "Unable to build the chained test fixture.\n");
    ms_free(&ms);
    return 1;
  }

  printf("Chained bitstream tests (%d links)\n", CHAIN_LINKS);

  /* Fixture sanity, read straight out of the Ogg pages. */
  for(i = 0; i < CHAIN_LINKS; i++) eos_granulepos[i] = -1;
  CHECK_I64(inspect_chain(&ms, eos_granulepos), 0);
  for(i = 0; i < CHAIN_LINKS; i++)
    CHECK_I64(eos_granulepos[i], chain_samples[i]);

  ms.pos = 0;
  cb = seekable_callbacks();
  CHECK_I64(ov_open_callbacks(&ms, &vf, NULL, 0, cb), 0);
  if(!checks_failed){
    test_structure(&vf);
    test_pcm_boundaries(&vf);
    test_time_boundaries(&vf);
    test_page_seeks(&vf);
    test_finite_tail(&vf);
    test_seekable_sequential(&vf);
  }
  CHECK_I64(ov_clear(&vf), 0);

  test_unseekable(&ms);

  ms_free(&ms);

  if(checks_failed){
    fprintf(stderr, "%d of %d chain checks failed.\n",
            checks_failed, checks_run);
    return 1;
  }
  printf("All %d chain checks passed.\n", checks_run);
  return 0;
}
