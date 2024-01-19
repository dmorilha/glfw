/*****************************************************************************
 * Title:   GLBoing
 * Desc:    Tribute to Amiga Boing.
 * Author:  Jim Brooks  <gfx@jimbrooks.org>
 *          Original Amiga authors were R.J. Mical and Dale Luck.
 *          GLFW conversion by Marcus Geelnard
 * Notes:   - 360' = 2*PI [radian]
 *
 *          - Distances between objects are created by doing a relative
 *            Z translations.
 *
 *          - Although OpenGL enticingly supports alpha-blending,
 *            the shadow of the original Boing didn't affect the color
 *            of the grid.
 *
 *          - [Marcus] Changed timing scheme from interval driven to frame-
 *            time based animation steps (which results in much smoother
 *            movement)
 *
 * History of Amiga Boing:
 *
 * Boing was demonstrated on the prototype Amiga (codenamed "Lorraine") in
 * 1985. According to legend, it was written ad-hoc in one night by
 * R. J. Mical and Dale Luck. Because the bouncing ball animation was so fast
 * and smooth, attendees did not believe the Amiga prototype was really doing
 * the rendering. Suspecting a trick, they began looking around the booth for
 * a hidden computer or VCR.
 *****************************************************************************/

#include <mutex>
#include <thread>

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "gl.h"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <linmath.h>

#include <pulse/pulseaudio.h>

#define DEBUG_TIME 1

const size_t SINE_HZ = 220; // not working
const int32_t SAMPLE_HZ = 44100;

static pa_sample_spec sample_specifications;

bool paused = true;

int volume = 0;

template <typename T>
static const T * sawtooth(T * const data, const int length, const int freq) {
  static T t = 0;
#if DEBUG_TIME
  static timeval t_zero;
  static bool first = true;
  if (first) {
    first = false;
    pa_gettimeofday(&t_zero);
  }
#endif
  for (int i = 0; i < length; i++) {
    // const float j = sin(((double) i / SAMPLE_HZ) * 2 * M_PI * SINE_HZ) / 2;
    data[i] = t * volume;
    t += 2; //* freq;
  }
#if DEBUG_TIME
  const auto usec = pa_bytes_to_usec(length, &sample_specifications);
  const auto age = pa_timeval_age(&t_zero);
  // printf("-> %f %f %li\n", data[length - 1], t, age);
#endif
  return data;
}

void stream_success_cb(pa_stream *stream, int success, void *userdata) {
  return;
}

void context_state_cb(pa_context* context, void* mainloop) {
  pa_threaded_mainloop_signal(reinterpret_cast<pa_threaded_mainloop*>(mainloop), 0);
}

void stream_state_cb(pa_stream *s, void *mainloop) {
  pa_threaded_mainloop_signal(reinterpret_cast<pa_threaded_mainloop*>(mainloop), 0);
}

void stream_write_cb(pa_stream *stream, size_t requested_bytes, void *userdata) {
  while (0 < requested_bytes) {
    uint8_t * buffer;
    size_t bytes_to_write = requested_bytes;
    pa_stream_begin_write(stream, reinterpret_cast<void**>(&buffer), &bytes_to_write);
    pa_stream_write(stream, sawtooth(buffer, bytes_to_write, SINE_HZ), bytes_to_write, nullptr, 0LL, PA_SEEK_RELATIVE);
    requested_bytes -= bytes_to_write;
  }
}

static void sync_playback_test() {
  pa_threaded_mainloop *mainloop;
  pa_mainloop_api *mainloop_api;
  pa_context *context;
  pa_stream *stream;

  // Get a mainloop and its context
  mainloop = pa_threaded_mainloop_new();
  assert(mainloop);
  mainloop_api = pa_threaded_mainloop_get_api(mainloop);
  context = pa_context_new(mainloop_api, "smily");
  assert(context);

  // Set a callback so we can wait for the context to be ready
  pa_context_set_state_callback(context, &context_state_cb, mainloop);

  // Lock the mainloop so that it does not run and crash before the context is ready
  pa_threaded_mainloop_lock(mainloop);

  // Start the mainloop
  assert(pa_threaded_mainloop_start(mainloop) == 0);
  assert(pa_context_connect(context, nullptr, PA_CONTEXT_NOAUTOSPAWN, nullptr) == 0);

  // Wait for the context to be ready
  for(;;) {
    pa_context_state_t context_state = pa_context_get_state(context);
    assert(PA_CONTEXT_IS_GOOD(context_state));
    if (context_state == PA_CONTEXT_READY) break;
    pa_threaded_mainloop_wait(mainloop);
  }

  sample_specifications.format = PA_SAMPLE_U8;
  sample_specifications.rate = SAMPLE_HZ;
  sample_specifications.channels = 1;
#if 0
  for (const auto & rate : {8000, 11025, 16000, 22050, 44100, 48000}) {
  sample_specifications.rate = rate;
  printf("pa bytes per second %li\n", pa_bytes_per_second(&sample_specifications));
  printf("pa frame size %li\n", pa_frame_size(&sample_specifications));
  printf("pa sample size %li\n", pa_sample_size(&sample_specifications));
  }
#endif

  stream = pa_stream_new(context, "Playback", &sample_specifications, nullptr);
  pa_stream_set_state_callback(stream, stream_state_cb, mainloop);
  pa_stream_set_write_callback(stream, stream_write_cb, mainloop);

  pa_buffer_attr buffer_attr;
  buffer_attr.maxlength = (uint32_t) -1;
  buffer_attr.tlength = 512;
  buffer_attr.prebuf = (uint32_t) -1;
  buffer_attr.minreq = 512;

  // Settings copied as per the chromium browser source
  pa_stream_flags_t stream_flags;

  stream_flags = static_cast<pa_stream_flags_t>(PA_STREAM_START_CORKED | PA_STREAM_INTERPOLATE_TIMING
  | PA_STREAM_NOT_MONOTONIC | PA_STREAM_AUTO_TIMING_UPDATE | PA_STREAM_ADJUST_LATENCY);

  // Connect stream to the default audio output sink
  assert(pa_stream_connect_playback(stream, nullptr, &buffer_attr, stream_flags, nullptr, nullptr) == 0);

  // Wait for the stream to be ready
  for(;;) {
    pa_stream_state_t stream_state = pa_stream_get_state(stream);
    assert(PA_STREAM_IS_GOOD(stream_state));
    if (PA_STREAM_READY == stream_state) {
      break;
    }
    pa_threaded_mainloop_wait(mainloop);
  }

  pa_threaded_mainloop_unlock(mainloop);

  // Uncork the stream so it will start playing
  pa_stream_cork(stream, 0, stream_success_cb, mainloop);
}

#define RADIUS           70.f
#define STEP_LONGITUDE   5.625f                   /* 22.5 makes 16 bands like original Boing */
#define STEP_LATITUDE    5.625f

#define DIST_BALL       (RADIUS * 2.f + RADIUS * 0.1f)

#define VIEW_SCENE_DIST (DIST_BALL * 3.f + 200.f)/* distance from viewer to middle of boing area */
#define GRID_SIZE       (RADIUS * 4.5f)          /* length (width) of grid */
#define BOUNCE_HEIGHT   (RADIUS * 2.1f)
#define BOUNCE_WIDTH    (RADIUS * 2.1f)

#define SHADOW_OFFSET_X -50.f
#define SHADOW_OFFSET_Y 50.f
#define SHADOW_OFFSET_Z -50.f

#define WALL_L_OFFSET   0.f
#define WALL_R_OFFSET   5.f

/* Animation speed (50.0 mimics the original GLUT demo speed) */
#define ANIMATION_SPEED 50.f

/* Maximum allowed delta time per physics iteration */
#define MAX_DELTA_T 0.02f

/* Draw ball, or its shadow */
typedef enum { DRAW_BALL, DRAW_BALL_SHADOW } DRAW_BALL_ENUM;

/* Vertex type */
typedef struct {float x; float y; float z;} vertex_t;

/* Global vars */
int windowed_xpos, windowed_ypos, windowed_width, windowed_height;
int width, height;
GLfloat deg_rot_y       = 0.f;
GLfloat deg_rot_y_inc   = 2.f;
int override_pos        = GLFW_FALSE;
GLfloat start_cursor_x  = 0.f;
GLfloat start_cursor_y  = 0.f;
GLfloat cursor_x        = 0.f;
GLfloat cursor_y        = 0.f;
GLfloat ball_x          = 0.f;
GLfloat ball_y          = 0.f;
GLfloat ball_x_inc      = 1.f;
GLfloat ball_y_inc      = 2.f;
DRAW_BALL_ENUM drawBallHow;
double  t;
double  t_old = 0.f;
double  dt;

/* Random number generator */
#ifndef RAND_MAX
#define RAND_MAX 4095
#endif


GLfloat TruncateDeg(GLfloat deg) {
  if (deg >= 360.f)
    return (deg - 360.f);
  else
    return deg;
}

double deg2rad(double deg) {
  return deg / 360 * (2 * M_PI);
}

double sin_deg(double deg){
  return sin(deg2rad(deg));
}

double cos_deg(double deg) {
  return cos(deg2rad(deg));
}

void CrossProduct(vertex_t a, vertex_t b, vertex_t c, vertex_t * n) {
  GLfloat u1, u2, u3;
  GLfloat v1, v2, v3;

  u1 = b.x - a.x;
  u2 = b.y - a.y;
  u3 = b.y - a.z;

  v1 = c.x - a.x;
  v2 = c.y - a.y;
  v3 = c.z - a.z;

  n->x = u2 * v3 - v2 * u3;
  n->y = u3 * v1 - v3 * u1;
  n->z = u1 * v2 - v1 * u2;
}

void init(void) {
  glClearColor(0.55f, 0.55f, 0.55f, 0.f);
  glShadeModel(GL_FLAT);
}

void BounceBall(double delta_t) {
  GLfloat sign;
  GLfloat deg;

  /* Bounce on walls */
  if (ball_x >  (BOUNCE_WIDTH / 2 + WALL_R_OFFSET)) {
    ball_x_inc = -0.5f - 0.75f * (GLfloat)rand() / (GLfloat)RAND_MAX;
    deg_rot_y_inc = -deg_rot_y_inc;
  }

  if ( ball_x < -(BOUNCE_HEIGHT/2 + WALL_L_OFFSET) ) {
    ball_x_inc =  0.5f + 0.75f * (GLfloat)rand() / (GLfloat)RAND_MAX;
    deg_rot_y_inc = -deg_rot_y_inc;
  }

  /* Bounce on floor / roof */
  if (ball_y >  BOUNCE_HEIGHT / 2) {
    ball_y_inc = -0.75f - 1.f * (GLfloat)rand() / (GLfloat)RAND_MAX;
  }
  if (ball_y < -BOUNCE_HEIGHT / 2 * 0.85) {
    ball_y_inc =  0.75f + 1.f * (GLfloat)rand() / (GLfloat)RAND_MAX;
  }

  /* Update ball position */
  ball_x += ball_x_inc * ((float)delta_t * ANIMATION_SPEED);
  ball_y += ball_y_inc * ((float)delta_t * ANIMATION_SPEED);

  /*
   * Simulate the effects of gravity on Y movement.
   */
  if (ball_y_inc < 0) {
    sign = -1.0;
  } else {
    sign = 1.0;
  }

  deg = (ball_y + BOUNCE_HEIGHT / 2) * 90 / BOUNCE_HEIGHT;

  if (deg > 80) {
    deg = 80;
  }

  if (deg < 10) {
    deg = 10;
  }

  ball_y_inc = sign * 4.f * (float) sin_deg( deg );
}

void DrawBoingBallBand(GLfloat long_lo, GLfloat long_hi, int i) {
  vertex_t vert_ne;
  vertex_t vert_nw;
  vertex_t vert_sw;
  vertex_t vert_se;
  vertex_t vert_norm;
  GLfloat lat_deg;

  int j = 0;
  /*
   * Iterate through the points of a latitude circle.
   * A latitude circle is a 2D set of X,Z points.
   */
  for (lat_deg = 0; lat_deg <= (360 - STEP_LATITUDE); lat_deg += STEP_LATITUDE) {

    if (drawBallHow == DRAW_BALL_SHADOW) {
      glColor3f( 0.35f, 0.35f, 0.35f );

    } else if (((i < 19 && i > 15) && (j == 11 || j == 12 || j == 18 || j == 19)) || ((i == 11 || i == 12) && (j > 8 && j < 22))) {
      glColor3f(0.f, 0.f, 0.f);
    } else {
      glColor3f( 255.0f, 255.0f, 0.0f );
    }

    /*
     * Assign each Y.
     */
    vert_ne.y = vert_nw.y = (float) cos_deg(long_hi) * RADIUS;
    vert_sw.y = vert_se.y = (float) cos_deg(long_lo) * RADIUS;

    /*
     * Assign each X,Z with sin,cos values scaled by latitude radius indexed by longitude.
     * Eg, long=0 and long=180 are at the poles, so zero scale is sin(longitude),
     * while long=90 (sin(90)=1) is at equator.
     */
    vert_ne.x = (float)cos_deg(lat_deg) * (RADIUS * (float)sin_deg(long_lo + STEP_LONGITUDE));
    vert_se.x = (float)cos_deg(lat_deg) * (RADIUS * (float)sin_deg(long_lo));
    vert_nw.x = (float)cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float)sin_deg(long_lo + STEP_LONGITUDE ));
    vert_sw.x = (float)cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float)sin_deg(long_lo));

    vert_ne.z = (float)sin_deg(lat_deg) * (RADIUS * (float)sin_deg(long_lo + STEP_LONGITUDE));
    vert_se.z = (float)sin_deg(lat_deg) * (RADIUS * (float)sin_deg(long_lo));
    vert_nw.z = (float)sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float)sin_deg(long_lo + STEP_LONGITUDE));
    vert_sw.z = (float)sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float)sin_deg(long_lo));

    if (override_pos && (i == 11 && j > 8 && j < 22)) {
      vert_se.x = (float)cos_deg(lat_deg) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      vert_sw.x = (float)cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      vert_se.z = (float)sin_deg(lat_deg) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      vert_sw.z = (float)sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      cursor_y = std::max(cursor_y, static_cast<GLfloat>(40));
      vert_sw.y = vert_se.y = (float)cos_deg(long_lo) + (cursor_y * 0.5f);
    }

    glBegin(GL_POLYGON);

    CrossProduct(vert_ne, vert_nw, vert_sw, &vert_norm);
    glNormal3f(vert_norm.x, vert_norm.y, vert_norm.z);
    glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z);
    glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z);
    glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z);
    glVertex3f(vert_se.x, vert_se.y, vert_se.z);

    if (drawBallHow == DRAW_BALL && override_pos && (((i < 19 && i > 15) && (j == 11 || j == 12 || j == 18 || j == 19)) || ((i == 11 || i == 12) && (j > 8 && j < 22)))) {
#if 0
      vert_se.z = (float)sin_deg(lat_deg) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      vert_sw.z = (float)sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
      vert_sw.y = vert_se.y = (float) cos_deg(long_lo - cursor_y * 0.1f) * RADIUS + rand() % 10;
#endif

      if (i == 18 && (j == 11 || j == 12 || j == 18 || j == 19)) {
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z);
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);
      }

      if ((i == 12 && j > 8 && j < 22)) {
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z);
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);
      }

      if (i == 16 && (j == 11 || j == 12 || j == 18 || j == 19)) {
        glVertex3f(vert_se.x, vert_se.y, vert_se.z);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);
      }

      if (i == 11 && j > 8 && j < 22) {
        vert_se.x = (float)cos_deg(lat_deg) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
        vert_sw.x = (float)cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * (float) sin_deg(long_lo - cursor_y * 0.1f));
        // vert_sw.y -= cursor_y;
        // vert_se.y -= cursor_y;
        // vert_sw.y = vert_se.y = (float)cos_deg(long_lo) + cursor_y * 0.1f;

        glVertex3f(vert_se.x, vert_se.y, vert_se.z);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);

      }

      if ((i < 19 && i > 15) && (j == 11 || j == 18)) {
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z);
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);
      }

      if (((i == 11 || i == 12)) && j == 9) {
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z);
        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);
      }

      if ((i < 19 && i > 15) && (j == 12 || j == 19)) {
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10 + cursor_y * 0.1f);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10 + cursor_y * 0.1f);
      }

      if ((i == 11 || i == 12) && j == 21) {
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);

        glVertex3f(vert_ne.x, vert_ne.y, vert_ne.z + 10);
        glVertex3f(vert_nw.x, vert_nw.y, vert_nw.z + 10);
        glVertex3f(vert_sw.x, vert_sw.y, vert_sw.z + 10);
        glVertex3f(vert_se.x, vert_se.y, vert_se.z + 10);
      }
    }

    glEnd();
    ++j;
  }

  return;
}


void DrawBoingBall(void) {
  GLfloat lon_deg;     /* degree of longitude */
  double dt_total, dt2;

  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);

  /*
   * Another relative Z translation to separate objects.
   */
  glTranslatef(0.0, 0.0, DIST_BALL);

  if ( ! paused) {
    dt_total = dt;
    while (dt_total > 0.0) {
      dt2 = dt_total > MAX_DELTA_T ? MAX_DELTA_T : dt_total;
      dt_total -= dt2;
      BounceBall(dt2);
      deg_rot_y = TruncateDeg( deg_rot_y + deg_rot_y_inc*((float)dt2*ANIMATION_SPEED) );
    }
  }

  /* Set ball position */
  glTranslatef(ball_x, ball_y, 0.0);

  if (drawBallHow == DRAW_BALL_SHADOW) {
    glTranslatef(SHADOW_OFFSET_X, SHADOW_OFFSET_Y, SHADOW_OFFSET_Z);
  }

  glRotatef(deg_rot_y, 1.0, 0.0, 0.0);
  glRotatef(deg_rot_y, 0.0, 1.0, 0.0);

  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);

  int i = 0;
  for (lon_deg = 0; lon_deg < 180; lon_deg += STEP_LONGITUDE) {
    DrawBoingBallBand(lon_deg, lon_deg + STEP_LONGITUDE, i++);
  }

  glPopMatrix();

  return;
}

void display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();

  drawBallHow = DRAW_BALL_SHADOW;
  DrawBoingBall();

  drawBallHow = DRAW_BALL;
  DrawBoingBall();

  glPopMatrix();
  glFlush();
}

void reshape(GLFWwindow * window, int w, int h) {
  mat4x4 projection, view;

  glViewport(0, 0, (GLsizei)w, (GLsizei)h);

  glMatrixMode(GL_PROJECTION);

  mat4x4_perspective(projection, 2.f * (float)atan2(RADIUS, 200.f), (float)w / (float)h, 1.f, VIEW_SCENE_DIST);
  glLoadMatrixf((const GLfloat*) projection);

  glMatrixMode( GL_MODELVIEW );
  {
    vec3 eye = { 0.f, 0.f, VIEW_SCENE_DIST };
    vec3 center = { 0.f, 0.f, 0.f };
    vec3 up = { 0.f, -1.f, 0.f };
    mat4x4_look_at( view, eye, center, up );
  }
  glLoadMatrixf((const GLfloat*) view);
}

void key_callback(GLFWwindow * window, int key, int scancode, int action, int mods) {
  static bool polygon_line_mode = false;

  if (action != GLFW_PRESS)
    return;

  if (key == GLFW_KEY_ESCAPE && mods == 0)
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  else if (key == GLFW_KEY_SPACE)
    paused = ! paused;

  else if (key == GLFW_KEY_W) {
    polygon_line_mode = ! polygon_line_mode;
    glPolygonMode(GL_FRONT_AND_BACK, polygon_line_mode ? GL_LINE : GL_FILL);


  } else if ((key == GLFW_KEY_ENTER && mods == GLFW_MOD_ALT) ||
      (key == GLFW_KEY_F11 && mods == GLFW_MOD_ALT)) {
    if (glfwGetWindowMonitor(window)) {
      glfwSetWindowMonitor(window, nullptr,
          windowed_xpos, windowed_ypos,
          windowed_width, windowed_height, 0);
    } else {
      GLFWmonitor * monitor = glfwGetPrimaryMonitor();
      if (monitor) {
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        glfwGetWindowPos(window, &windowed_xpos, &windowed_ypos);
        glfwGetWindowSize(window, &windowed_width, &windowed_height);
        glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
      }
    }
  }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
  // printf("mouse button callback %li\n", std::this_thread::get_id());
  if (button != GLFW_MOUSE_BUTTON_LEFT)
    return;
  override_pos = action == GLFW_PRESS ? GLFW_TRUE : GLFW_FALSE;
  volume = action == GLFW_PRESS ? 1.0 : 0;
}

void cursor_position_callback( GLFWwindow* window, double x, double y )
{
  if (override_pos == GLFW_FALSE) {
    start_cursor_x = (float)x;
    start_cursor_y = (float)y;
  } else {
    cursor_x = (float)x - start_cursor_x;
    cursor_y = (float)y - start_cursor_y;
  }
}

void scroll_callback(GLFWwindow* window, double x, double y)
{
#if 0
  zoom += (float) y / 4.f;
  if (zoom < 0)
    zoom = 0;
#endif
  // deg_rot_y += y * 4.f;
}

int main() {
  sync_playback_test();
  GLFWwindow * window = nullptr;

  /* Init GLFW */
  if ( ! glfwInit())
    exit(EXIT_FAILURE);

  window = glfwCreateWindow(400, 400, "smily", nullptr, nullptr);
  if ( ! window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  glfwSetWindowAspectRatio(window, 1, 1);
  glfwSetFramebufferSizeCallback(window, reshape);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  glfwSetScrollCallback(window, scroll_callback);

  glfwMakeContextCurrent(window);
  gladLoadGL(glfwGetProcAddress);
  glfwSwapInterval(1);

  glfwGetFramebufferSize(window, &width, &height);
  reshape(window, width, height);

  glfwSetTime(0.0);

  init();

  /* Main loop */
  for (;;) {
    /* Timing */
    t = glfwGetTime();
    dt = t - t_old;
    t_old = t;

    display();
    glfwSwapBuffers(window);

    /* Swap buffers */
    glfwPollEvents();

    /* Check if we are still running */
    if (glfwWindowShouldClose(window)) {
      break;
    }

  }

  glfwTerminate();
  return 0;
}

