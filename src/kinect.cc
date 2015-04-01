#ifndef BUILDING_NODE_EXTENSION
#define BUILDING_NODE_EXTENSION
#endif // BUILDING_NODE_EXTENSION

#include <v8.h>
#include "node.h"
#include "node_buffer.h"
#include "uv.h"

extern "C" {
  #include <libfreenect/libfreenect.h>
}

#include <stdio.h>
#include <stdexcept>
#include <string>

#include "kinect.h"

using namespace node;
using namespace v8;


namespace
{
    v8::Persistent<v8::String> depthCallbackSymbol;
    v8::Persistent<v8::String> videoCallbackSymbol;

    void video_callback(freenect_device *, void *, uint32_t);
    void async_video_callback(uv_async_t *, int);

    void depth_callback(freenect_device *, void *, uint32_t);
    void async_depth_callback(uv_async_t *, int);

    kinect::Context *get_kinect_context(uv_async_t *);
    kinect::Context *get_kinect_context(freenect_device *);

    void throw_message(char const *);
}


namespace kinect {

  Context *
  Context::GetContext(const Arguments &args) {
    return ObjectWrap::Unwrap<Context>(args.This());
  }

  /********************************/
  /********* Flow *****************/
  /********************************/

  void
  process_event_thread(void *arg) {
    Context * context = (Context *) arg;
    while(context->running_) {
      freenect_process_events(context->context_);
    }
  }

  void
  Context::InitProcessEventThread() {
    uv_thread_create(&event_thread_, process_event_thread, this);
  }

  void
  Context::Resume() {
    if (! running_) {
      running_ = true;
      InitProcessEventThread();
    }
  }

  Handle<Value>
  Context::Resume(const Arguments& args) {
    GetContext(args)->Resume();
    return Undefined();
  }

  void
  Context::Pause() {
    if (running_) {
      running_ = false;
      uv_thread_join(&event_thread_);
    }
  }

  Handle<Value>
  Context::Pause(const Arguments& args) {
    GetContext(args)->Pause();
    return Undefined();
  }


    // =====================================================================
    // = Video                                                             =
    // =====================================================================

    Handle<Value> Context::SetVideoCallback(const Arguments& args)
    {
        HandleScope scope;
        GetContext(args)->SetVideoCallback();
        return Undefined();
    }

    void Context::SetVideoCallback()
    {
        if (!videoCallback_)
        {
            freenect_set_video_callback(device_, video_callback);

            videoCallback_ = true;
            if (freenect_set_video_mode(device_, videoMode_) != 0)
            {
                throw_message("Error setting Video mode");
                return;
            }

            videoBuffer_ = Buffer::New(videoMode_.bytes);
            videoBufferPersistentHandle_ = Persistent<Value>::New(videoBuffer_->handle_);
            if (freenect_set_video_buffer(device_, Buffer::Data(videoBuffer_)) != 0)
            {
                throw_message("Error setting Video buffer");
            }

        }

        if (freenect_start_video(device_) != 0)
        {
            throw_message("Error starting video");
            return;
        }
    }

    void Context::VideoCallback()
    {
        sending_ = true;
        assert(videoBuffer_ != nullptr);

        if (videoCallbackSymbol.IsEmpty())
        {
            videoCallbackSymbol = NODE_PSYMBOL("videoCallback");
        }

        Local<Value> callback_v =handle_->Get(videoCallbackSymbol);

        if (!callback_v->IsFunction())
        {
            throw_message("VideoCallback should be a function");
        }

        Local<Function> callback = Local<Function>::Cast(callback_v);

        Handle<Value> argv[1] = { videoBuffer_->handle_ };
        callback->Call(handle_, 1, argv);
        sending_ = false;
    }



  /********* Depth ****************/



  void
  Context::DepthCallback() {
    sending_ = true;
    if (depthCallbackSymbol.IsEmpty()) {
      depthCallbackSymbol = NODE_PSYMBOL("depthCallback");
    }
    Local<Value> callback_v =handle_->Get(depthCallbackSymbol);
    if (!callback_v->IsFunction()) {
      ThrowException(Exception::Error(String::New("depthCallback should be a function")));
    }
    Local<Function> callback = Local<Function>::Cast(callback_v);
    Handle<Value> argv[1] = { depthBuffer_->handle_ };
    callback->Call(handle_, 1, argv);
    sending_ = false;
  }

  void
  Context::SetDepthCallback() {
    if (!depthCallback_) {
      depthCallback_ = true;
      freenect_set_depth_callback(device_, depth_callback);
      if (freenect_set_depth_mode(device_, depthMode_) != 0) {
        ThrowException(Exception::Error(String::New("Error setting depth mode")));
        return;
      };

      // Allocate buffer

      depthBuffer_ = Buffer::New(depthMode_.bytes);
      depthBufferPersistentHandle_ = Persistent<Value>::New(depthBuffer_->handle_);
      if (freenect_set_depth_buffer(device_, Buffer::Data(depthBuffer_)) != 0) {
        ThrowException(Exception::Error(String::New("Error setting depth buffer")));
      }

    }

    if (freenect_start_depth(device_) != 0) {
      ThrowException(Exception::Error(String::New("Error starting depth")));
      return;
    }
  }

  Handle<Value>
  Context::SetDepthCallback(const Arguments& args) {

    GetContext(args)->SetDepthCallback();

    return Undefined();
  }

  /**** Tilt Control *****/

  void
  Context::Tilt(const double angle) {
    freenect_set_tilt_degs(device_, angle);
  }

  Handle<Value>
  Context::Tilt(const Arguments& args) {
    HandleScope scope;

    if (args.Length() == 1) {
      if (!args[0]->IsNumber()) {
        return ThrowException(Exception::TypeError(
          String::New("tilt argument must be a number")));
      }
    } else {
      return ThrowException(Exception::Error(String::New("Expecting at least one argument with the led status")));
    }

    double angle = args[0]->ToNumber()->NumberValue();
    GetContext(args)->Tilt(angle);
    return Undefined();
  }

  /**** LED Control ******/

  void
  Context::Led(const std::string option) {
    freenect_led_options ledCode;

    if (option == "off") {
      ledCode = LED_OFF;
    } else if (option == "green") {
      ledCode = LED_GREEN;
    } else if (option == "red") {
      ledCode = LED_RED;
    } else if (option == "yellow") {
      ledCode = LED_YELLOW;
    } else if (option == "blink green") {
      ledCode = LED_BLINK_GREEN;
    } else if (option == "blink red yellow") {
      ledCode = LED_BLINK_RED_YELLOW;
    } else {
      ThrowException(Exception::Error(String::New("Did not recognize given led code")));
      return;
    }

    if (freenect_set_led(device_, ledCode) < 0) {
      ThrowException(Exception::Error(String::New("Error setting led")));
      return;
    }
  }

  Handle<Value>
  Context::Led(const Arguments& args) {
    HandleScope scope;

    if (args.Length() == 1) {
      if (!args[0]->IsString()) {
        return ThrowException(Exception::TypeError(
          String::New("led argument must be a string")));
      }
    } else {
      return ThrowException(Exception::Error(String::New("Expecting at least one argument with the led status")));
    }

    String::AsciiValue val(args[0]->ToString());
    GetContext(args)->Led(std::string(*val));
    return Undefined();
  }


  /********* Life Cycle ***********/

  Handle<Value>
  Context::New(const Arguments& args) {
    HandleScope scope;

    assert(args.IsConstructCall());

    int user_device_number = 0;
    if (args.Length() == 1) {
      if (!args[0]->IsNumber()) {
        return ThrowException(Exception::TypeError(
          String::New("user_device_number must be an integer")));
      }
      user_device_number = (int) args[0]->ToInteger()->Value();
      if (user_device_number < 0) {
        return ThrowException(Exception::RangeError(
          String::New("user_device_number must be a natural number")));
      }
    }

    Context *context = new Context(user_device_number);
    context->Wrap(args.This());

    return args.This();
  }

  void
  Context::Initialize(v8::Handle<v8::Object> target) {
    HandleScope scope;

    Local<FunctionTemplate> t = FunctionTemplate::New(New);
    t->InstanceTemplate()->SetInternalFieldCount(1);

    NODE_SET_PROTOTYPE_METHOD(t, "close", Close);
    NODE_SET_PROTOTYPE_METHOD(t, "led",   Led);
    NODE_SET_PROTOTYPE_METHOD(t, "tilt",   Tilt);
    NODE_SET_PROTOTYPE_METHOD(t, "setDepthCallback", SetDepthCallback);
    NODE_SET_PROTOTYPE_METHOD(t, "setVideoCallback", SetVideoCallback);
    NODE_SET_PROTOTYPE_METHOD(t, "pause",            Pause);
    NODE_SET_PROTOTYPE_METHOD(t, "resume",           Resume);

    target->Set(String::NewSymbol("Context"), t->GetFunction());
  }

  Context::Context(int user_device_number) : ObjectWrap() {
    context_         = NULL;
    device_          = NULL;
    depthCallback_   = false;
    videoCallback_   = false;
    running_         = false;

    if (freenect_init(&context_, NULL) < 0) {
      ThrowException(Exception::Error(String::New("Error initializing freenect context")));
      return;
    }

    freenect_set_log_level(context_, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(context_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
    int nr_devices = freenect_num_devices (context_);
    if (nr_devices < 1) {
      Close();
      ThrowException(Exception::Error(String::New("No kinect devices present")));
      return;
    }

    if (freenect_open_device(context_, &device_, user_device_number) < 0) {
      Close();
      ThrowException(Exception::Error(String::New("Could not open device number\n")));
      return;
    }

    freenect_set_user(device_, this);

    // Initialize video mode
    videoMode_ = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
    assert(videoMode_.is_valid);

    // Initialize depth mode
    depthMode_ = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
    assert(depthMode_.is_valid);

    // LibUV stuff
    uv_loop_t *loop = uv_default_loop();
    uv_async_init(loop, &uv_async_video_callback_, async_video_callback);
    uv_async_init(loop, &uv_async_depth_callback_, async_depth_callback);
  }

  void
  Context::Close() {

    running_ = false;

    if (device_ != NULL) {
      if (freenect_close_device(device_) < 0) {
        ThrowException(Exception::Error(String::New(("Error closing device"))));
        return;
      }

      device_ = NULL;
    }

    if (context_ != NULL) {
      if (freenect_shutdown(context_) < 0) {
        ThrowException(Exception::Error(String::New(("Error shutting down"))));
        return;
      }

      context_ = NULL;
    }
  }

  Context::~Context() {
    Close();
  }

  Handle<Value>
  Context::Close(const Arguments& args) {
    HandleScope scope;
    GetContext(args)->Close();
    return Undefined();
   }

  void
  Initialize(Handle<Object> target) {
    Context::Initialize(target);
  }

}


namespace
{
    kinect::Context *get_kinect_context(uv_async_t *const handle)
    {
        auto const context = static_cast<kinect::Context *>(handle->data);
        assert(context != nullptr);
        return context;
    }

    kinect::Context *get_kinect_context(freenect_device *const device)
    {
        auto const context = static_cast<kinect::Context *>(
                freenect_get_user(device));
        assert(context != nullptr);
        return context;
    }

    void async_video_callback(uv_async_t *handle, int notUsed)
    {
        auto const context = get_kinect_context(handle);
        context->VideoCallback();
    }

    void video_callback(freenect_device *dev, void *video, uint32_t timestamp)
    {
        auto const context = get_kinect_context(dev);
        if (context->sending_)
            return;
        context->uv_async_video_callback_.data = (void *) context;
        uv_async_send(&context->uv_async_video_callback_);
    }

    void async_depth_callback(uv_async_t *handle, int notUsed)
    {
        auto const context = get_kinect_context(handle);
        if (context->sending_)
            return;
        context->DepthCallback();
    }

    void depth_callback(freenect_device *dev, void *depth, uint32_t timestamp)
    {
        auto const context = get_kinect_context(dev);
        context->uv_async_depth_callback_.data = (void *) context;
        uv_async_send(&context->uv_async_depth_callback_);
    }

    void throw_message(char const *const message)
    {
        ThrowException(Exception::Error(String::New(message)));
    }
}


extern "C" void
init(Handle<Object> target)
{
  kinect::Initialize(target);
}

NODE_MODULE(kinect, init)

