#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <sys/time.h>

/****************************************************************************/
// Misc

#define MAX(a,b)      ((a) > (b) ? (a) : (b))
#define MIN(a,b)      ((a) < (b) ? (a) : (b))

static int verbose;
static int shouting;
static int spiking;
float slowdown=1;

#define SAND_SIZE (8*1024*1024)     // must be power of 2
#define MUSEC_SEC 1000000LL

typedef long long tick_t;
typedef long long musec_t;

static tick_t ticks_per_sec = 0;
static musec_t sleep_granularity = 0;
static musec_t start_musec = 0;

typedef enum
  {A_NONE, A_SLEEP , A_MUL   , A_FMUL  , A_ADD   , A_MEMORY, A_PAUSE, A_MUL_FMUL, A_DIV2, A_DIV8209, A_MEMW0, A_MEMW1, NUM_ACTIVITY} activity_t;
const int activity_shout_freq[NUM_ACTIVITY] =
//{0     , 100     , 200     , 220 0   , 1400    ,  600    , 1000    };
  {0     , 262     , 294     , 330     , 370     ,  415    , 466     , 0        , 0     , 0        , 0      , 0       }; // , 2093
#define SHOUT_OCTAVE 4
     

static inline musec_t get_time_musec();

/****************************************************************************/
// Logging and reporting

void error(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
  fputc('\n', stderr);
  exit(1);
}


void say(const char *format, ...) {
  if (verbose) {
    long long stamp = get_time_musec() - start_musec;
    va_list ap;
    printf("%lld.%06lld: ", stamp/MUSEC_SEC, stamp%MUSEC_SEC);
    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);
    putchar('\n');
  }
}


#define MAX_LOG_ENTRIES 100000

struct log_entry {
  const char* activity;
  musec_t start_musec;
  musec_t end_musec;
};

static struct log_entry log_data[MAX_LOG_ENTRIES];
static unsigned int log_length = 0;

static void tell_log(const struct log_entry *entry) {
  if (log_length>=MAX_LOG_ENTRIES)
    error("Log overflow");
  log_data[log_length] = *entry;
  ++log_length;
}

static void dump_log() {
  unsigned int i;
  for (i=0; i<log_length; ++i) {
    printf("> %12.9f %12.9f %12.9f  %s\n",
       ((double)(log_data[i].start_musec-start_musec))/MUSEC_SEC,
       ((double)(log_data[i].end_musec-start_musec))/MUSEC_SEC,
       ((double)(log_data[i].end_musec-log_data[i].start_musec))/MUSEC_SEC,
       log_data[i].activity);
  }
}


/****************************************************************************/
// PC speaker spikes
#if defined(__i386__)
#define PIT_FREQ 1193180.0 /* Hz */
#define COUNT_SEC 0.001 // max: 0.054
#define COUNT_TICKS ((short)(PIT_FREQ * COUNT_SEC))
#define PIT_MODE 0

static void spike(activity_t act) {
    /* Set mode register of the 8253 PIT:
       10 11  ??? 0  = B0 | (PIT_MODE<<1)
           ^^ ^^  ^^^ ^
           |  |    |  \-- count in binary, not BCD
           |  |    \-- mode (0=low until timeout; 3=square wave)
           |  \-- write least significant byte first, then most significant byte
           \-- select counter 2
       (http://www.boondog.com/%5Ctutorials%5C8254%5Charris8254.pdf) */
    outb_p(0xB0 | (PIT_MODE<<1),0x43);
    /* Load counter 2 of the 8253 PIT, LSB then MSB: */
    outb_p(COUNT_TICKS & 0xff, 0x42);
    outb((COUNT_TICKS>>8) & 0xff, 0x42);
}

static void init_spiking() {
    if ((ioperm(0x61,1,1)!=0) || (ioperm(0x40,4,1)!=0) || (ioperm(0x80,4,1)!=0)) {
        perror("ioperm: ");
        exit(1);
    }
    /* Tell 8255 chip to enable the 8253's input gate line (bit 0)
       and the PC speaker's AND gate (bit 1): */
    outb_p(inb_p(0x61) | 0x03, 0x61);
}

static void cleanup_spiking() {
    /* Quiet! */
    outb_p(inb_p(0x61) & ~0x03, 0x61);
}
#else
static void spike(activity_t act) {}
static void init_spiking() {}
static void cleanup_spiking() {}
#endif

/****************************************************************************/
// Shouting

#define DSP_FILENAME "/dev/dsp"
#define DSP_FORMAT_T signed short // signed char
#define DSP_FORMAT_AFMT AFMT_S16_LE // AFMT_S8
#define DSP_FORMAT_UNSIGNED 0
#define SHOUT_BUF_SEC 1/1
static int dsp_channels = 1;
static int dsp_freq = 8000;

#define DSP_FORMAT_RANGE ((1<<(sizeof(DSP_FORMAT_T)*8))-1)
#define FLOAT_TO_SAMPLE(x) ((DSP_FORMAT_T)( ((x)+DSP_FORMAT_UNSIGNED)/2*DSP_FORMAT_RANGE ))
static int dsp_fd = -1;
static int shout_buf_cur_sample = 0;
static int shout_samples_pending = 0;
static activity_t shout_activity = A_NONE;
static int dsp_buf_bytes = -1; // OSS's buffer in bytes
static musec_t dsp_buf_musec = -1; // .. and in microseconds.
static musec_t dsp_refill_musec = -1;
static tick_t dsp_refill_ticks = -1;
static int shout_buf_bytes = -1;
static int shout_buf_samples = -1;
static DSP_FORMAT_T *(shout_bufs[NUM_ACTIVITY]);


// Returns a normally distributed deviate with zero mean and unit variance.
// Based on gasdev() from Numerical Recipes in C.
float gaussian_random() {
  static int iset=0;
  static float gset;
  float fac,rsq,v1,v2;
  if (iset == 0) {
    do {
      v1=2.0*random()/RAND_MAX-1.0;
      v2=2.0*random()/RAND_MAX-1.0;
      rsq=v1*v1+v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac=sqrt(-2.0*log(rsq)/rsq);
    gset=v1*fac;
    iset=1;
    return v2*fac;
  } else {
    iset=0;
    return gset;
  }
}

//static void set_dsp_format() {
//  int i;
//  audio_buf_info info;
//
//  i = DSP_FORMAT_AFMT;
//  if (ioctl(dsp_fd, SNDCTL_DSP_SETFMT, &i)<0 || i!=DSP_FORMAT_AFMT)
//    error("SNDCTL_DSP_SETFMT failed: %s", strerror(errno));
//
//  --dsp_channels;
//  if (ioctl(dsp_fd, SNDCTL_DSP_STEREO, &dsp_channels)<0)
//    error("SNDCTL_DSP_STEREO failed: %s", strerror(errno));
//  ++dsp_channels;
//
//  if (ioctl(dsp_fd, SNDCTL_DSP_SPEED, &dsp_freq)<0)
//    error("SNDCTL_DSP_SPEED failed: %s", strerror(errno));
//
//  if (ioctl(dsp_fd, SNDCTL_DSP_GETOSPACE, &info)<0)
//    error("SNDCTL_DSP_GETOSPACE failed: %s\n", strerror(errno));
//
//  dsp_buf_bytes = MIN(info.bytes, info.fragstotal*info.fragsize);
//  dsp_buf_musec = dsp_buf_bytes*MUSEC_SEC/(sizeof(DSP_FORMAT_T)*dsp_channels*dsp_freq);
//  dsp_refill_musec = dsp_buf_musec * 9/10;
//  dsp_refill_ticks = dsp_refill_musec*ticks_per_sec/MUSEC_SEC;
//  printf("* Using %d audio channel(s) at %dHz, %dbit.\n* OSS buffer is %d*%d=%d bytes, can write %d (hence: %fsec).\n* Will refill every %fsec.\n", dsp_channels, dsp_freq, (int)sizeof(DSP_FORMAT_T)*8, info.fragstotal, info.fragsize, info.fragstotal*info.fragsize, info.bytes, 1.0*dsp_buf_musec/MUSEC_SEC, 1.0*dsp_refill_musec/MUSEC_SEC);
//}

//static void init_shouting() {
//  int a, i, j;
//  if (!shouting)
//    return;
//  dsp_fd = open(DSP_FILENAME, O_WRONLY | O_NONBLOCK);
//  if (dsp_fd<0)
//    error("Cannot open %s: %s", DSP_FILENAME, strerror(errno));
//  // Default format is 8-bit unsigned mono 8KHz */
//
//  set_dsp_format();
//  shout_buf_samples = dsp_freq * dsp_channels * SHOUT_BUF_SEC;
//  shout_buf_bytes = shout_buf_samples * (dsp_channels*sizeof(DSP_FORMAT_T));
//  printf("* Shouting uses buffers of %d samples (%d bytes)\n", shout_buf_samples, shout_buf_bytes);
//
//  for (a=0; a<NUM_ACTIVITY; ++a) {
//    if ((shout_bufs[a] = (DSP_FORMAT_T*)malloc(shout_buf_bytes))==NULL)
//      error("init_shouting: malloc failed");
//    for (i=0; i<shout_buf_samples; ++i) {
//      for (j=0; j<dsp_channels; ++j) {
//    double k = 1.0/dsp_freq * activity_shout_freq[a]*pow(2,SHOUT_OCTAVE) * 2*M_PI;
//    shout_bufs[a][i*dsp_channels+j] = FLOAT_TO_SAMPLE(0.95*sin(i*k));
//      }
//    }
//  }
//  for (i=0; i<shout_buf_samples; ++i) {
//    for (j=0; j<dsp_channels; ++j) {
//      shout_bufs[A_NONE][i*dsp_channels+j] = FLOAT_TO_SAMPLE( MIN(1,MAX(-1,gaussian_random()/3)) );
//    }
//  }
//  printf("* Shouting buffers initialized (%d bytes total).\n", NUM_ACTIVITY*shout_buf_bytes);
//}

//static void crank_shouting() {
//  if (!shouting || shout_samples_pending<=0)
//    return;
//  do {
//    int bytes_to_write, bytes_written, samples_written, stop=0;
//    if (shout_samples_pending <= 0)
//      return;
//    if (shout_buf_cur_sample>=shout_buf_samples)
//      shout_buf_cur_sample = 0;
//    bytes_to_write = MIN(shout_samples_pending, shout_buf_samples-shout_buf_cur_sample)
//                     * sizeof(DSP_FORMAT_T) * dsp_channels;
//
//    audio_buf_info info;
//    if (ioctl(dsp_fd, SNDCTL_DSP_GETOSPACE, &info)<0)
//      error("SNDCTL_DSP_GETOSPACE failed: %s\n", strerror(errno));
//    // info.bytes == #bytes that can be written without blocking
//    if (info.bytes<bytes_to_write) {
//      // printf("* Wanted to write %d bytes, room for only %d\n", bytes_to_write, info.bytes);
//      stop=1;
//      if (info.bytes==0)
//    return;
//      bytes_to_write = info.bytes;
//    }
//
//    bytes_written = write(dsp_fd,
//              shout_bufs[shout_activity] + shout_buf_cur_sample*dsp_channels,
//              bytes_to_write);
//    //printf("* %f: Wrote %d bytes starting at sample %d, act=%d\n", 1.0*(get_time_musec()-start_musec)/MUSEC_SEC, bytes_written, shout_buf_cur_sample, shout_activity);
//    if (bytes_written<0) {
//      if (errno==EAGAIN)
//    return;
//      else if (errno==EINTR)
//    continue;
//      else
//    error("Error while cranking shouting: %s", strerror(errno));
//    }
//    assert(bytes_written==bytes_to_write);
//    assert(bytes_written % (sizeof(DSP_FORMAT_T)*dsp_channels) == 0);
//    samples_written = bytes_written/(sizeof(DSP_FORMAT_T)*dsp_channels);
//    //printf("* crank_shouting: written=%d\n", written);
//    shout_buf_cur_sample += samples_written;
//    shout_samples_pending -= samples_written;
//    if (stop)
//      return;
//  } while (shout_samples_pending>0);
//  if (ioctl(dsp_fd, SNDCTL_DSP_POST, 0)<0)
//    error("SNDCTL_DSP_POST failed: %s", strerror(errno));
//}

//static void start_shouting(activity_t act, float sec) {
//  double max_sec;
//  if (!shouting)
//    return;
//  assert(shout_activity==A_NONE);
//  max_sec = (sec + 2.0*sleep_granularity/MUSEC_SEC) * 1.1; // avoid buffer underruns
//  shout_samples_pending = (int)(dsp_freq*max_sec);
//  shout_buf_cur_sample = 0; // eek!
//  shout_activity = act;
//  //printf("* shout: will play %d samples\n", shout_samples_pending);
//  crank_shouting();
//}
//
//static void stop_shouting() {
//  if (!shouting)
//    return;
//  if (verbose && shout_samples_pending>0)
//    printf("WARNING: when stopping activity #%d, %d shouting samples not yet written\n", shout_activity, shout_samples_pending);
//  shout_activity=A_NONE;
//  if (ioctl(dsp_fd, SNDCTL_DSP_RESET, 0)<0)
//    error("SNDCTL_DSP_RESET failed: %s", strerror(errno));
//    // Note: SNDCTL_DSP_RESET may forget format settings (and does on Cygwin)
//  //printf("* quiet\n");
//  //set_dsp_format(0);
//}


/****************************************************************************/
// Timing

#if defined(__arm64__)
#define rdtscll(val) {\
    struct timeval tv;\
    if (gettimeofday(&tv, NULL)!=0)\
      error("gettimeofday failed: %s", strerror(errno));\
    val = tv.tv_sec*MUSEC_SEC + tv.tv_usec;\
  }
#elif defined(__i386__)
#define rdtscll(val) asm volatile("rdtsc" : "=A" (val))
#else
#error "rdtsc not defined to this arch"
#endif

static inline musec_t get_time_musec() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL)!=0)
    error("gettimeofday failed: %s", strerror(errno));
  return tv.tv_sec*MUSEC_SEC + tv.tv_usec;
}



static int sand[SAND_SIZE];

#define DEFINE_LOOP(NAME,PRECODE,CODE) \
static void loop_##NAME(tick_t end_tick) \
{ \
  tick_t now_tick; rdtscll(now_tick); \
  PRECODE; \
  while (now_tick < end_tick) { \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; CODE; \
    rdtscll(now_tick); \
  } \
}  \
static void perform_##NAME(float sec) { \
  struct log_entry entry; tick_t first_tick, last_tick, next_tick; \
  say(#NAME); entry.activity = #NAME;  \
  if (spiking) spike(A_##NAME); \
  rdtscll(first_tick); \
  last_tick = first_tick + (tick_t)(ticks_per_sec*sec*slowdown); \
  entry.start_musec = get_time_musec(); \
  loop_##NAME(last_tick); \
  entry.end_musec = get_time_musec(); \
  tell_log(&entry); \
}

#if defined(__arm64__)
DEFINE_LOOP(PAUSE, {},  asm volatile ("nop"));
DEFINE_LOOP(ADD, {}, asm volatile ("add w0, w0, #0" : : : "w0") );
DEFINE_LOOP(MUL, {}, asm volatile ("mul w0, w1, w2" : : : "w0", "w1", "w2") );
#ifndef __ANDROID_API__
//DEFINE_LOOP(FMUL, {}, asm volatile ("VMUL.F32 s14, s14, s15" : : : "s14", "s15") );
//DEFINE_LOOP(MUL_FMUL, {}, asm volatile ("mul w0, w1, w2; VMUL.F32 s14, s14, s15" : : : "w0", "w1", "w2", "s14", "s15") );
#endif
#elif defined(__i386__)
DEFINE_LOOP(PAUSE, {},  asm volatile ("rep;nop"));
DEFINE_LOOP(ADD, {}, asm volatile ("addl $0, %%eax" : : : "eax") );
DEFINE_LOOP(MUL, {}, asm volatile ("mull %%edx" : : : "eax", "edx") );
DEFINE_LOOP(FMUL, {}, asm volatile ("fmulp") );
DEFINE_LOOP(MUL_FMUL, {}, asm volatile ("mull %%edx; fmulp" : : : "eax", "edx") );
#else
#error "loops not defined for this arch"
#endif
DEFINE_LOOP(MEMORY, static long i=0; volatile int x, i = (i+257) & (SAND_SIZE-1); x = sand[i]; );
DEFINE_LOOP(DIV2, volatile int y=2; volatile int res, res = 0x77777777/y );
DEFINE_LOOP(DIV8209, volatile int y=2; volatile int res, res = 0x77777777/y );
DEFINE_LOOP(MEMW0, static long i=0, sand[i+3]=sand[i+2]=sand[i+1]=sand[i]= 0; i = (i+4) & (SAND_SIZE-1); );
DEFINE_LOOP(MEMW1, static long i=0, sand[i+3]=sand[i+2]=sand[i+1]=sand[i]=-1; i = (i+4) & (SAND_SIZE-1); );


static long long get_ticks_per_sec(float calib_sec) {
  tick_t i=1;
  long long start_micro, end_micro;
  long long start_tick;
  printf("# Calibrating (%f)\n", calib_sec);
  do {
    i *= 2;
    start_micro = get_time_musec();
    rdtscll(start_tick);
    loop_PAUSE(start_tick+i);
    end_micro = get_time_musec();
  } while (end_micro-start_micro < calib_sec*MUSEC_SEC);
  printf("# Done\n");
  return i*1000000L/(end_micro-start_micro);
}

static void coarse_sleep(musec_t total_musec, musec_t *start_musec, musec_t *end_musec) {
  musec_t start = get_time_musec();
  musec_t current = start;
  musec_t end = start+total_musec;

  while (current<end) {
    // wake up in time to crank shouting
    musec_t desired = (dsp_refill_musec>0) ? MIN(end-current, dsp_refill_musec) : end-current;
    struct timespec req, rem;
    req.tv_sec = desired/MUSEC_SEC;
    req.tv_nsec = (desired%MUSEC_SEC)*1000;
    while (1) {
      //printf("nanosleep (req=%d.%09d)\n", (int)req.tv_sec, (int)req.tv_nsec);
      int err = nanosleep(&req,&rem);
      if (err==0)
    break;
      else if (err==EINTR)
    req = rem;
      else
    error("nanosleep failed (req=%d.%09d): %s", req.tv_sec, req.tv_nsec, strerror(errno));
    }
//    crank_shouting();
    current = get_time_musec();
  }
  if (start_musec)
    *start_musec = start;
  if (end_musec)
    *end_musec = current;
}

static void perform_sleep(float sec) {
  struct log_entry entry;
  tick_t now;
  say("SLEEP");
  entry.activity = "SLEEP";
//  start_shouting(A_SLEEP, sec*slowdown);
  if (spiking)
    spike(A_SLEEP);
  rdtscll(now);
  coarse_sleep((musec_t)(sec*slowdown*MUSEC_SEC), &entry.start_musec, &entry.end_musec);
//  stop_shouting();
  if (verbose>1)
    say("  (wanted %f, took %f)", sec*slowdown, 1.0*(entry.end_musec-entry.start_musec)/MUSEC_SEC);
  tell_log(&entry);
}

static musec_t get_sleep_granularity() {
  musec_t start, end;
  coarse_sleep(100, NULL, NULL);
  coarse_sleep(100, &start, &end);
  return end-start;
}


/****************************************************************************/
// Main


int main(int argc, char **argv) {
  /*** INIT ***/
  int i, trial=0, watch=0, shortcalib=0, whitenoise=0, justmem=0, justmul=0, justpause=0, mulsleep=0, mulfmul=0, quiet=0, exotic=0;
  float d;
  verbose=0;
  shouting=0;
  spiking=0;
  
  for (i=1; i<argc; ++i) {
    if (strcmp(argv[i],"-t")==0) {
      trial = 1; verbose = 1; shortcalib=1;
    } else if  (strcmp(argv[i],"-w")==0) {
      watch = 1; verbose = 1; shortcalib=1;
    } else if(strcmp(argv[i],"-v")==0) {
      verbose++;
    } else if(strcmp(argv[i],"--slow")==0) {
      slowdown*=2;
    } else if(strcmp(argv[i],"--exotic")==0) {
      exotic=1;
    } else if(strcmp(argv[i],"-q")==0) {
      quiet = 1;
    } else if (strcmp(argv[i],"-s")==0) {
      shouting = 1;
    } else if (strcmp(argv[i],"--spike")==0) {
      spiking = 1;
    } else if (strcmp(argv[i],"-f")==0) {
      dsp_freq = 48000; // ask for fast dsp rate when someone skimped on the a low-pass filter
    } else if (strcmp(argv[i],"-F")==0) {
      dsp_freq = 96000;
    } else if (strcmp(argv[i],"--whitenoise")==0) {
      whitenoise=1; shouting=1; shortcalib=1;
    } else if (strcmp(argv[i],"--justmem")==0) {
      justmem=1; shortcalib=1;
    } else if (strcmp(argv[i],"--justmul")==0) {
      justmul=1; shortcalib=1;
    } else if (strcmp(argv[i],"--justpause")==0) {
      justpause=1; shortcalib=1;
    } else if (strcmp(argv[i],"--mulsleep")==0) {
      mulsleep=1; verbose=1; shortcalib=1;
    } else if (strcmp(argv[i],"--mulfmul")==0) {
      mulfmul=1; verbose=1; shortcalib=1;
    } else
      error("Unknown command-line argument: %s", argv[i]);
  }
  if (watch)
    puts("# Watch mode");
  if (shouting)
    puts("# Shouting!");
  if (spiking) {
    puts("# Spiking!");
    init_spiking();
  } if (quiet)
    verbose=0;
  if (slowdown>0)
    printf("# Slowdown=%f\n", slowdown);

  start_musec = get_time_musec();
  ticks_per_sec = get_ticks_per_sec(shortcalib ? 0.3 : 3);
  sleep_granularity = get_sleep_granularity();
  printf("TPS: %lld    Sleep granularity: %fsec\n", ticks_per_sec, 1.0*sleep_granularity/MUSEC_SEC);
  //  set_realtime_sched();
  memset(sand, 0xFF, SAND_SIZE*sizeof(int));
//  init_shouting(); // must happen after ticks_per_sec is calibrated
  coarse_sleep(1, NULL, NULL); // align to clock boundary

  /** GO ***/
  if (whitenoise) {
    printf("! White noise !\n");
    while (1) {
//      start_shouting(A_NONE, 1000);
      coarse_sleep(1000*MUSEC_SEC, 0, 0);
    }
  } else if (justmem) {
    puts("! Memory only");
    while (1)
      perform_MEMORY(1000);
  } else if (justmul) {
    puts("! MUL only");
    while (1)
      perform_MUL(1000);
  } else if (mulfmul) {
#ifndef __ANDROID_API__
    puts("! MUL/FMUL/MUL_FMUL only");
    while (1) {
      int i;
      perform_MUL(1);
//      perform_FMUL(1);
//      perform_MUL_FMUL(1);
      for (i=0; i<500; ++i) {
    perform_MUL(0.001);
//    perform_FMUL(0.001);
      }
      for (i=0; i<500; ++i) {
    perform_MUL(0.0005);
//    perform_FMUL(0.0015);
      }
      perform_sleep(0.2);
    }
#else
    printf("not supported\n");
#endif
  } else if (trial) {
    puts("! Trial");
    while (1) {
      perform_MUL(0.8);
      perform_sleep(0.4);
      perform_MEMORY(0.8);
      perform_sleep(0.4);
    }
  } else if (mulsleep) {
    puts("! MUL/SLEEP");
    while (1) {
      perform_MUL(0.4);
      perform_sleep(0.2);
    }
  } else {
    if (!watch) {
      perform_MUL(30);
      perform_sleep(30);
    }

    if (1) {
      d = (watch ? 1.0 : 8.0);
      do {
    if (verbose)
      printf("# Activity set of %f sec\n", d);
    perform_MUL(d);
#ifndef __ANDROID_API__
//    perform_FMUL(d);
#endif
    perform_ADD(d);
    perform_MEMORY(d);
    perform_PAUSE(d);
    if (exotic) {
      perform_DIV2(d);
      perform_DIV8209(d);
      perform_MEMW0(d);
      perform_MEMW1(d);
    }
    perform_sleep(d);

    d /= ( (d>0.5) ? 1.5 : (d>0.03) ? 1.07 : 1.02);
      } while (d>0.00001);
    }

    if (1) {
      d = 0.65925;
      do {
    if (verbose)
      printf("# Resolution set of %f sec\n", d);
    for (i=0; i < (watch ? 3 : 10); ++i) {
      perform_MUL(d);
      perform_sleep(d/2);
    }
    d /= ( (d>0.1) ? 1.5 : (d>0.01) ? 1.3 : 1.1);
      } while (d>0.00001);
    }
  
    if (1) {
      for (i=0; i<0; ++i) {
    perform_MUL(0.01);
    perform_sleep(0.005);
      }
    }
  
  }
//  stop_shouting();
  if (spiking)
    cleanup_spiking();
  dump_log();
  return 0;
}

/*
  coarse_sleep(d);
  loop_PAUSE(d);
  loop_ADD(d);
  loop_MUL(d);
*/

