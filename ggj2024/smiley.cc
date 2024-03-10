/*
 * Inspired by glfw's GLBoing example
 *
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
 */

#include <memory>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <linmath.h>

#include "gl.h"

#define DEBUG_TIME 0
#define PROCESS_NAME "smiley"
#define PULSE_AUDIO 1

#if PULSE_AUDIO
#include <pulse/pulseaudio.h>

const size_t SINE_HZ = 220; // not working
const int32_t SAMPLE_HZ = 44100;

static pa_sample_spec sample_specifications;

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
    data[i] = t * volume;
    t += 2; //* freq;
  }
#if DEBUG_TIME
  const auto usec = pa_bytes_to_usec(length, &sample_specifications);
  const auto age = pa_timeval_age(&t_zero);
  printf("-> %f %f %li\n", data[length - 1], t, age);
#endif
  return data;
}

void stream_success_cb(pa_stream *, int, void *) {
  return;
}

void context_state_cb(pa_context *, void * const mainloop) {
  assert(nullptr != mainloop);
  pa_threaded_mainloop_signal(reinterpret_cast<pa_threaded_mainloop*>(mainloop), 0);
}

void stream_state_cb(pa_stream *, void * const mainloop) {
  assert(nullptr != mainloop);
  pa_threaded_mainloop_signal(reinterpret_cast<pa_threaded_mainloop*>(mainloop), 0);
}

void stream_write_cb(pa_stream * const stream, size_t requested_bytes, void *) {
  assert(nullptr != stream);
  while (0 < requested_bytes) {
    uint8_t * buffer;
    size_t bytes_to_write = requested_bytes;
    pa_stream_begin_write(stream, reinterpret_cast<void * *>(&buffer), &bytes_to_write);
    pa_stream_write(stream, sawtooth(buffer, bytes_to_write, SINE_HZ), bytes_to_write, nullptr, 0LL, PA_SEEK_RELATIVE);
    requested_bytes -= bytes_to_write;
  }
}

void start_pulse_audio() {
  pa_threaded_mainloop * mainloop;
  pa_mainloop_api * mainloop_api;
  pa_context * context;
  pa_stream * stream;

  // Get a mainloop and its context
  mainloop = pa_threaded_mainloop_new();
  assert(mainloop);
  mainloop_api = pa_threaded_mainloop_get_api(mainloop);
  context = pa_context_new(mainloop_api, PROCESS_NAME);
  assert(context);

  // Set a callback so we can wait for the context to be ready
  pa_context_set_state_callback(context, &context_state_cb, mainloop);

  // Lock the mainloop so that it does not run and crash before the context is ready
  pa_threaded_mainloop_lock(mainloop);

  // Start the mainloop
  assert(pa_threaded_mainloop_start(mainloop) == 0);
  assert(pa_context_connect(context, nullptr, PA_CONTEXT_NOAUTOSPAWN, nullptr) == 0);

  // Wait for the context to be ready
  while (true) {
    pa_context_state_t context_state = pa_context_get_state(context);
    assert(PA_CONTEXT_IS_GOOD(context_state));
    if (context_state == PA_CONTEXT_READY) break;
    pa_threaded_mainloop_wait(mainloop);
  }

  sample_specifications.format = PA_SAMPLE_U8;
  sample_specifications.rate = SAMPLE_HZ;
  sample_specifications.channels = 1;

  stream = pa_stream_new(context, PROCESS_NAME, &sample_specifications, nullptr);
  pa_stream_set_state_callback(stream, stream_state_cb, mainloop);
  pa_stream_set_write_callback(stream, stream_write_cb, mainloop);

  pa_buffer_attr buffer_attr;
  buffer_attr.maxlength = static_cast<uint32_t>(-1);
  buffer_attr.tlength = 512;
  buffer_attr.prebuf = static_cast<uint32_t>(-1);
  buffer_attr.minreq = 512;

  // Settings copied as per the chromium browser source
  pa_stream_flags_t stream_flags;

  stream_flags = static_cast<pa_stream_flags_t>(PA_STREAM_START_CORKED | PA_STREAM_INTERPOLATE_TIMING
    | PA_STREAM_NOT_MONOTONIC | PA_STREAM_AUTO_TIMING_UPDATE | PA_STREAM_ADJUST_LATENCY);

  // Connect stream to the default audio output sink
  assert(pa_stream_connect_playback(stream, nullptr, &buffer_attr, stream_flags, nullptr, nullptr) == 0);

  // Wait for the stream to be ready
  while (true) {
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
#endif // PULSE_AUDIO

// Global vars
int windowed_xpos, windowed_ypos, windowed_width, windowed_height;

bool mouth_action = false;
bool movement = false;

float cursor_x = 0;
float cursor_y = 0;

int zoom = 200;

template <typename T>
constexpr T TruncateDegree(const T degree) {
  if (degree >= 360) {
    return (degree - 360);
  } else {
    return degree;
  }
}

template <typename T>
constexpr T deg2rad(const T deg) {
  return deg / 360 * (2 * M_PI);
}

template <typename T>
T sin_deg(const T deg){
  return sin(deg2rad(deg));
}

template <typename T>
T cos_deg(const T deg) {
  return cos(deg2rad(deg));
}

struct Vertex {
  float x = 0;
  float y = 0;
  float z = 0;
};

static constexpr float RADIUS = 70;
static constexpr float DISTANCE_BALL = RADIUS * 2 + RADIUS * 0.1;

struct Smiley {
  static constexpr float ANIMATION_SPEED = 100;
  static constexpr float STEP_LONGITUDE = 5.625;
  static constexpr float STEP_LATITUDE = 5.625;
  // distance from viewer to middle of boing area
  // length (width) of grid
  static constexpr float GRID_SIZE = RADIUS * 4.5;

  static constexpr float MAX_DELTA_T = 0.02;

  int height_ = RADIUS * 2.1;
  int width_ = RADIUS * 2.1;

  float ball_x = 0;
  float ball_x_inc = 1;
  float ball_y = 0;
  float ball_y_inc = 2;

  float rotation_x = 0;
  float rotation_y = 0;

  bool animation_paused = true;

  void BounceBall(const double delta_t) {
    // Bounce on walls
    if (ball_x >  (width_ / 2)) {
      ball_x_inc = -0.5 - 0.75 * static_cast<float>(rand() / RAND_MAX);
      // deg_rot_y_inc = -deg_rot_y_inc;
    }

    if (ball_x < -(height_ / 2)) {
      ball_x_inc =  0.5 + 0.75 * static_cast<float>(rand() / RAND_MAX);
      // deg_rot_y_inc = -deg_rot_y_inc;
    }

    /* Bounce on floor / roof */
    if (ball_y >  height_ / 2) {
      ball_y_inc = -0.75 - 1 * static_cast<float>(rand() / RAND_MAX);
    }
    if (ball_y < -height_ / 2 * 0.85) {
      ball_y_inc =  0.75 + 1 * static_cast<float>(rand() / RAND_MAX);
    }

    // Update ball position
    ball_x += ball_x_inc * delta_t * ANIMATION_SPEED;
    ball_y += ball_y_inc * delta_t * ANIMATION_SPEED;

    // Simulate the effects of gravity on Y movement.
    const float sign = 0 > ball_y_inc ? -1 : 1;
    float degree = (ball_y + height_ / 2) * 90 / height_;

    if (80 < degree) {
      degree = 80;
    }

    if (10 > degree) {
      degree = 10;
    }

    ball_y_inc = sign * 4 * sin_deg(degree);
  }

  constexpr bool eye(const int i) {
    return 18 >= i && 16 <= i;
  }

  constexpr bool leftEye(const int i, const int j) {
    return eye(i) && (11 == j || 12 == j);
  }

  constexpr bool rightEye(const int i, const int j) {
    return eye(i) && (18 == j || 19 == j);
  }

  constexpr bool mouth(const int i, const int j) {
    return (11 == i || 12 == i) && (9 <= j && 21 >= j);
  }

  void DrawBoingBallBand(float long_lo, float long_hi, int i) {
    struct {
      Vertex northeast, northwest, southeast, southwest;
    } vertex;

    int j = 0;
    /*
     * Iterate through the points of a latitude circle.
     * A latitude circle is a 2D set of X,Z points.
     */
    for (float lat_deg = 0; lat_deg <= 360 - STEP_LATITUDE; lat_deg += STEP_LATITUDE) {
      if (leftEye(i, j) || rightEye(i, j) || mouth(i, j)) {
        glColor3f(0, 0, 0);
      } else {
        glColor3f(255, 255, 0);
      }

      // Assign each Y.
      vertex.northeast.y = vertex.northwest.y = cos_deg(long_hi) * RADIUS;
      vertex.southwest.y = vertex.southeast.y = cos_deg(long_lo) * RADIUS;

      /*
       * Assign each X,Z with sin,cos values scaled by latitude radius indexed by longitude.
       * Eg, long=0 and long=180 are at the poles, so zero scale is sin(longitude),
       * while long=90 (sin(90)=1) is at equator.
       */
      vertex.northeast.x = cos_deg(lat_deg) * (RADIUS * sin_deg(long_lo + STEP_LONGITUDE));
      vertex.southeast.x = cos_deg(lat_deg) * (RADIUS * sin_deg(long_lo));
      vertex.northwest.x = cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo + STEP_LONGITUDE ));
      vertex.southwest.x = cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo));

      vertex.northeast.z = sin_deg(lat_deg) * (RADIUS * sin_deg(long_lo + STEP_LONGITUDE));
      vertex.southeast.z = sin_deg(lat_deg) * (RADIUS * sin_deg(long_lo));
      vertex.northwest.z = sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo + STEP_LONGITUDE));
      vertex.southwest.z = sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo));

      if (mouth_action && 11 == i && mouth(i, j)) {
        vertex.southeast.x = cos_deg(lat_deg) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));
        vertex.southwest.x = cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));
        vertex.southeast.z = sin_deg(lat_deg) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));
        vertex.southwest.z = sin_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));
        cursor_y = std::max(cursor_y, static_cast<float>(40));
        vertex.southwest.y = vertex.southeast.y = cos_deg(long_lo) + (cursor_y * 0.5);
      }

      glBegin(GL_POLYGON);

      glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z);
      glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z);
      glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z);
      glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z);

      if (mouth_action && (leftEye(i, j) || rightEye(i, j) || mouth(i, j))) {

        if (18 == i && (leftEye(i, j) || rightEye(i, j))) {
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z);
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);
        }

        if (12 == i && mouth(i, j)) {
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z);
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);
        }

        if (16 == i && (leftEye(i, j) || rightEye(i, j))) {
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);
        }

        if (11 == i && mouth(i, j)) {
          vertex.southeast.x = cos_deg(lat_deg) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));
          vertex.southwest.x = cos_deg(lat_deg + STEP_LATITUDE) * (RADIUS * sin_deg(long_lo - cursor_y * 0.1));

          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);
        }

        if ((11 == j || 18 ==  j) && eye(i)) {
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z);
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);
        }

        if (9 == j && mouth(i, j)) {
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z);
          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);
        }

        if ((12 == j || 19 == j) && eye(i)) {
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10 + cursor_y * 0.1);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10 + cursor_y * 0.1);
        }

        if (21 == j && mouth(i, j)) {
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);

          glVertex3f(vertex.northeast.x, vertex.northeast.y, vertex.northeast.z + 10);
          glVertex3f(vertex.northwest.x, vertex.northwest.y, vertex.northwest.z + 10);
          glVertex3f(vertex.southwest.x, vertex.southwest.y, vertex.southwest.z + 10);
          glVertex3f(vertex.southeast.x, vertex.southeast.y, vertex.southeast.z + 10);
        }
      }

      glEnd();
      ++j;
    }

    return;
  }

  void DrawBoingBall(const double time) {
    double dt_total, dt2;

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);

    // Another relative Z translation to separate objects.
    glTranslatef(0, 0, DISTANCE_BALL);

    if ( ! animation_paused) {
      dt_total = time;
      while (0 < dt_total) {
        dt2 = dt_total > MAX_DELTA_T ? MAX_DELTA_T : dt_total;
        dt_total -= dt2;
        BounceBall(dt2);
      }
    }

    // Set ball position.
    glTranslatef(ball_x, ball_y, 0);

    glRotatef(rotation_x, 0, 1, 0);
    glRotatef(rotation_y, 1, 0, 0);

    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);

    int i = 0;
    for (float longitudeDegree = 0; longitudeDegree < 180; longitudeDegree += STEP_LONGITUDE) {
      DrawBoingBallBand(longitudeDegree, longitudeDegree + STEP_LONGITUDE, i++);
    }

    glPopMatrix();

    return;
  }

  void draw(const double time) {
    DrawBoingBall(time);
  }

  void togglePause() {
    animation_paused = ! animation_paused;
  }

  void resize(const int width, const int height) {
    height_ = height / 5;
    width_ = width / 3;
  }

  void rotateX(const float x) {
    rotation_x = TruncateDegree(x);
  }

  void rotateY(const float y) {
    rotation_y = TruncateDegree(y);
  }
};

void init(void) {
  glClearColor(0.55, 0.55, 0.55, 0);
  glShadeModel(GL_FLAT);
}

void display(const std::unique_ptr<Smiley> & smiley, const double time) {
  assert(static_cast<bool>(smiley));
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix();
  smiley->draw(time);
  glPopMatrix();
  glFlush();
}

void reshape(GLFWwindow * const window, const int width, const int height) {
  const float VIEW_SCENE_DISTANCE = DISTANCE_BALL * 3 + zoom;

  mat4x4 projection, view;

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);

  mat4x4_perspective(projection, 2 * atan2(RADIUS, 200), static_cast<float>(width) / height, 1, VIEW_SCENE_DISTANCE);
  glLoadMatrixf(reinterpret_cast<const float *>(projection));

  glMatrixMode(GL_MODELVIEW);

  {
    vec3 eye = {0, 0, VIEW_SCENE_DISTANCE};
    vec3 center = {0, 0, 0};
    vec3 up = {0, -1, 0};
    mat4x4_look_at(view, eye, center, up);
  }

  glLoadMatrixf(reinterpret_cast<const float *>(view));

  assert(nullptr != window);
  Smiley * const smiley = static_cast<Smiley *>(glfwGetWindowUserPointer(window));
  assert(nullptr != smiley);
  smiley->resize(width, height);
}

void key_callback(GLFWwindow * const window, int key, int scancode, int action, int mods) {
  static bool polygon_line_mode = false;
  assert(nullptr != window);
  Smiley * const smiley = static_cast<Smiley *>(glfwGetWindowUserPointer(window));
  assert(nullptr != smiley);

  if (GLFW_PRESS != action) {
    return;
  }

  if (GLFW_KEY_ESCAPE == key && 0 == mods) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  } else if (GLFW_KEY_SPACE == key) {
    smiley->togglePause();

  } else if (GLFW_KEY_W == key) {
    polygon_line_mode = ! polygon_line_mode;
    glPolygonMode(GL_FRONT_AND_BACK, polygon_line_mode ? GL_LINE : GL_FILL);

  } else if ((GLFW_KEY_ENTER == key && GLFW_MOD_ALT == mods) ||
      (GLFW_KEY_F11 == key && GLFW_MOD_ALT == mods)) {

    if (glfwGetWindowMonitor(window)) {
      glfwSetWindowMonitor(window, nullptr,
          windowed_xpos, windowed_ypos,
          windowed_width, windowed_height, 0);
    } else {
      GLFWmonitor * const monitor = glfwGetPrimaryMonitor();
      if (nullptr != monitor) {
        const GLFWvidmode * const mode = glfwGetVideoMode(monitor);
        glfwGetWindowPos(window, &windowed_xpos, &windowed_ypos);
        glfwGetWindowSize(window, &windowed_width, &windowed_height);
        glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
      }
    }
  }
}

void mouse_button_callback(GLFWwindow * const window, int button, int action, int mods) {
  if (GLFW_MOUSE_BUTTON_LEFT == button) {
    mouth_action = action == GLFW_PRESS;
#if PULSE_AUDIO
    volume = action == GLFW_PRESS ? 1 : 0;
#endif
  } else if (GLFW_MOUSE_BUTTON_RIGHT == button) {
    movement = action == GLFW_PRESS;
  }
}

void cursor_position_callback(GLFWwindow * const window, double x, double y) {
  static float start_cursor_x = 0;
  static float start_cursor_y = 0;

  assert(nullptr != window);
  Smiley * const smiley = static_cast<Smiley *>(glfwGetWindowUserPointer(window));
  assert(nullptr != smiley);

  if (mouth_action) {
    cursor_x = x - start_cursor_x;
    cursor_y = y - start_cursor_y;
  } else if (movement) {
    smiley->rotateX((x - start_cursor_x) * -1);
    smiley->rotateY((y - start_cursor_y) * -1);
  } else {
    start_cursor_x = x;
    start_cursor_y = y;
  }
}

void scroll_callback(GLFWwindow * window, double x, double y) {
  zoom += y * 25;
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  reshape(window, width, height);
}

int main(int argc, char * * argv) {
#if PULSE_AUDIO
  start_pulse_audio();
#endif
  GLFWwindow * window = nullptr;

  // Init GLFW
  if ( ! glfwInit()) {
    exit(EXIT_FAILURE);
  }

  window = glfwCreateWindow(400, 400, PROCESS_NAME, nullptr, nullptr);

  if ( ! window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  auto smiley = std::make_unique<Smiley>();
  glfwSetWindowUserPointer(window, smiley.get());

  glfwSetWindowAspectRatio(window, 1, 1);
  glfwSetFramebufferSizeCallback(window, reshape);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  glfwSetScrollCallback(window, scroll_callback);

  glfwMakeContextCurrent(window);
  gladLoadGL(glfwGetProcAddress);
  glfwSwapInterval(1);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  reshape(window, width, height);

  glfwSetTime(0.0);

  init();

  // game loop
  while (true) {
    static double past = 0;
    {
      const double now = glfwGetTime();
      const double time = now - past;
      past = now;
      display(smiley, time);
    }

    // Swap buffers
    glfwSwapBuffers(window);

    // Poll events
    glfwPollEvents();

    // Check if we are still running
    if (glfwWindowShouldClose(window)) {
      break;
    }
  }

  glfwTerminate();

  return 0;
}

