#ifndef _GNU_SOURCE
   #define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "VeyeCameraIsp.h"


#include "mipi327_cam_stream.h"



// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
//#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 2
#define SPLITTER_PREVIEW_PORT 1
#define SPLITTER_ENCODER_PORT 0

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MIPI_MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MIPI_MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MIPI_MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s


int mmal_status_to_int(MMAL_STATUS_T status);
static void signal_handler(int signal_number);

// Forward
typedef struct RASPIVID_STATE_S RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
   void (*nalCallback)(unsigned int len, unsigned char* nal);
} NALCallbackState;


typedef struct {
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
   void (*callback)(unsigned int w, unsigned int h, unsigned char * frame);
} YUVCallbackState ;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE_S
{
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   MMAL_FOURCC_T encoding;             /// Requested codec video encoding (MJPEG or H264)
   int bitrate;                        /// Requested bitrate
   int framerate;                      /// Requested frame rate (fps)
   int intraperiod;                    /// Intra-refresh period (key frame rate)
   int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
   int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
   int verbose;                        /// !0 if want detailed run information
   int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
                                       /// the camera output or the encoder output (with compression artifacts)
   int profile;                        /// H264 profile to use for encoding
   int level;                          /// H264 level to use for encoding

   VEYE_CAMERA_ISP_STATE	veye_camera_isp_state;

   MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
   MMAL_PORT_T *video_port;
   MMAL_CONNECTION_T *isp_connection; /// Pointer to the connection from isp to camera
   MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder
   MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port
   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   NALCallbackState nalState;        /// Used to move data to the encoder callback
   YUVCallbackState yuvState;

   int inlineMotionVectors;             /// Encoder outputs inline Motion Vectors
   int cameraNum;                       /// Camera number
   int sensor_mode;			            /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
   int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.

   MMAL_BOOL_T addSPSTiming;
   int slices;
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T  level_map[] =
{
   {"4",           MMAL_VIDEO_LEVEL_H264_4},
   {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
   {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);




/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(CamSettings camSet, RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   // Now set anything non-zero
   state->width = camSet.requestedWidth;
   state->height = camSet.requestedHeight;
   state->encoding = MMAL_ENCODING_H264;
   state->bitrate = camSet.bitrate;
   state->framerate = camSet.frameRate;
   state->intraperiod = 10;    // Not set
   state->quantisationParameter = 0;
   state->immutableInput = 1;
   state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   state->level = MMAL_VIDEO_LEVEL_H264_4;

   state->bInlineHeaders = 1;

   state->inlineMotionVectors = 0;
   state->cameraNum = -1;
   state->sensor_mode = 0;

   state->intra_refresh_type = -1;

   state->addSPSTiming = MMAL_FALSE;
   state->slices = 1;

   veye_camera_isp_set_defaults(&state->veye_camera_isp_state);
   state->veye_camera_isp_state.height_align = 16;
}


/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPIVID_STATE *state)
{
   int i;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   fprintf(stderr, "Width %d, Height %d\n", state->width, state->height);
   fprintf(stderr, "bitrate %d, framerate %d\n", state->bitrate, state->framerate);
   fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state->profile, profile_map, profile_map_size));
   fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state->level, level_map, level_map_size));
   fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state->quantisationParameter, state->bInlineHeaders ? "Yes" : "No");
   fprintf(stderr, "H264 Fill SPS Timings %s\n", state->addSPSTiming ? "Yes" : "No");
   fprintf(stderr, "H264 Slices %d\n", state->slices);
   fprintf(stderr, "\n\n");

}




static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   static int64_t base_time =  -1;
   static int64_t last_second = -1;

   YUVCallbackState *pData = (YUVCallbackState*)port->userdata;

   if (buffer->length)
   {
       mmal_buffer_header_mem_lock(buffer);
       //printf("Got yuv buf of %d bytes\n", buffer->length);
       pData->callback(port->format->es->video.width, port->format->es->video.height, buffer->data);
       mmal_buffer_header_mem_unlock(buffer);
   } else {
       printf("Empty buf\n");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}


/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   static int64_t base_time =  -1;
   static int64_t last_second = -1;

   // All our segment times based on the receipt of the first encoder callback
   if (base_time == -1)
      base_time = vcos_getmicrosecs64()/1000;

   // We pass our file handle and other stuff in via the userdata field.

   NALCallbackState *pData = (NALCallbackState *)port->userdata;

   //printf("H264 BUF %d\n", buffer->length);
   mmal_buffer_header_mem_lock(buffer);
   pData->nalCallback(buffer->length, buffer->data);
   mmal_buffer_header_mem_unlock(buffer);

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");

      //vcos_log_error("DONE buf %d ",buffer->length);
   }
}


/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE *state, 
                                               void (*callback)(unsigned int w, unsigned int h, unsigned char * frame))
{
   MMAL_COMPONENT_T *splitter = 0;
   MMAL_PORT_T *splitter_output = NULL;
   MMAL_ES_FORMAT_T *format;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL;
   int i;

   if (state->veye_camera_isp_state.isp_component == NULL)
   {
       status = MMAL_ENOSYS;
       vcos_log_error("Camera component must be created before splitter");
       if (splitter)
           mmal_component_destroy(splitter);
       return status;
   }

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

   if (status != MMAL_SUCCESS)
   {
       vcos_log_error("Failed to create splitter component");
       if (splitter)
           mmal_component_destroy(splitter);
       return status;
   }

   if (!splitter->input_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have any input port");
      if (splitter)
          mmal_component_destroy(splitter);
      return status;
   }

   vcos_log_error("Splitter has %d output port,you could use num 2,3 for extend", splitter->output_num);
   
   if (splitter->output_num < 2)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Splitter doesn't have enough output ports");
      if (splitter)
          mmal_component_destroy(splitter);
      return status;
   }

   /* Ensure there are enough buffers to avoid dropping frames: */
   mmal_format_copy(splitter->input[0]->format, state->veye_camera_isp_state.isp_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);

   if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   status = mmal_port_format_commit(splitter->input[0]);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on splitter input port");
      if (splitter)
          mmal_component_destroy(splitter);
      return status;
   }

   /* Splitter can do format conversions, configure format for its output port: */
   vcos_log_error(":: split it %d\n", splitter->output_num);
   for (i = 0; i < splitter->output_num; i++)
   {
       //printf("====> Conf splitter %d\n", i);
       mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);
       status = mmal_port_format_commit(splitter->output[i]);

       if (status != MMAL_SUCCESS)
       {
           vcos_log_error("Unable to set format on splitter output port %d", i);
           if (splitter)
               mmal_component_destroy(splitter);
           return status;
       }
   }

   status = mmal_component_enable(splitter);
   if (status != MMAL_SUCCESS)
   {
       vcos_log_error("splitter component couldn't be enabled");
       if (splitter)
           mmal_component_destroy(splitter);
       return status;
   }
   state->splitter_component = splitter;
   video_port = splitter->output[2];
   state->video_port = video_port;
   //format = video_port->format;
   //format->encoding_variant = MMAL_ENCODING_I420;
   ////format->encoding = MMAL_ENCODING_OPAQUE;
   //format->encoding = MMAL_ENCODING_I420;
   //format->es->video.width = VCOS_ALIGN_UP(state->width, 16);
   //format->es->video.height = VCOS_ALIGN_UP(state->height, 8);
   //format->es->video.crop.x = 0;
   //format->es->video.crop.y = 0;
   //format->es->video.crop.width = state->width;
   //format->es->video.crop.height = state->height;
   //format->es->video.frame_rate.num = state->framerate;
   //format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;
   //vcos_log_error("set encoder splitter out put frame rate  %d, w/h %d/%d ", state->framerate, state->width, state->height);
   //MMAL_PARAMETER_FRAME_RATE_T frame_t = {{MMAL_PARAMETER_FRAME_RATE,sizeof(MMAL_PARAMETER_FRAME_RATE_T)},{state->framerate,30}};
   //status = mmal_port_format_commit(video_port);
   //if (status != MMAL_SUCCESS)
   //{
   //   vcos_log_error("camera video format couldn't be set %d ",status);
   //   goto error;
   //} 
   //////////////
   YUVCallbackState* callback_data = & state->yuvState;
   callback_data->pstate = state;
   pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
   if (!pool)
   {
      vcos_log_error("POOL Failed to create buffer header pool for splitter output port %s", video_port->name);
   } 
   state->video_pool = pool;
   callback_data->callback = callback;
   video_port->userdata = (struct MMAL_PORT_USERDATA_T *)callback_data;
   status = mmal_port_enable(video_port, video_buffer_callback);
   if (status != MMAL_SUCCESS)
   {
       vcos_log_error("couldn't enamble splitter output port ",status);
       if (splitter)
           mmal_component_destroy(splitter);
       return status;
   } 
   int num = mmal_queue_length(pool->queue);
   for (int q=0;q<num;q++)
   {
       MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

       if (!buffer)
           vcos_log_error("Unable to get a required buffer %d from pool queue", q);

       if (mmal_port_send_buffer(video_port, buffer)!= MMAL_SUCCESS)
           vcos_log_error("Unable to send a buffer to camera output port (%d)", q);
   }


   if (state->verbose)
      fprintf(stderr, "Splitter component done\n");

   return status;

}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(RASPIVID_STATE *state)
{
   if (state->video_pool)
   {
      mmal_port_pool_destroy(state->video_port, state->video_pool);
   }

   if (state->splitter_component)
   {
      mmal_component_destroy(state->splitter_component);
      state->splitter_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to create video encoder component");
        if (encoder)
            mmal_component_destroy(encoder);
        state->encoder_component = NULL;
        return status;
    }

    if (!encoder->input_num || !encoder->output_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Video encoder doesn't have input/output ports");
        if (encoder)
            mmal_component_destroy(encoder);
        state->encoder_component = NULL;
        return status;
    }

    encoder_input = encoder->input[0];
    encoder_output = encoder->output[0];

    // We want same format on input and output
    mmal_format_copy(encoder_output->format, encoder_input->format);

    // Only supporting H264 at the moment
    encoder_output->format->encoding = state->encoding;

    if(state->encoding == MMAL_ENCODING_H264)
    {
        if(state->level == MMAL_VIDEO_LEVEL_H264_4)
        {
            if(state->bitrate > MIPI_MAX_BITRATE_LEVEL4)
            {
                fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
                state->bitrate = MIPI_MAX_BITRATE_LEVEL4;
            }
        }
        else
        {
            if(state->bitrate > MIPI_MAX_BITRATE_LEVEL42)
            {
                fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
                state->bitrate = MIPI_MAX_BITRATE_LEVEL42;
            }
        }
    }
    else if(state->encoding == MMAL_ENCODING_MJPEG)
    {
        if(state->bitrate > MIPI_MAX_BITRATE_MJPEG)
        {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->bitrate = MIPI_MAX_BITRATE_MJPEG;
        }
    }
   
    encoder_output->format->bitrate = state->bitrate;

    if (state->encoding == MMAL_ENCODING_H264)
        encoder_output->buffer_size = encoder_output->buffer_size_recommended;
    else
        encoder_output->buffer_size = 256<<10;


    if (encoder_output->buffer_size < encoder_output->buffer_size_min)
        encoder_output->buffer_size = encoder_output->buffer_size_min;

    encoder_output->buffer_num = encoder_output->buffer_num_recommended;

    if (encoder_output->buffer_num < encoder_output->buffer_num_min)
        encoder_output->buffer_num = encoder_output->buffer_num_min;

    // We need to set the frame rate on output to 0, to ensure it gets
    // updated correctly from the input framerate when port connected
    encoder_output->format->es->video.frame_rate.num = 0;
    encoder_output->format->es->video.frame_rate.den = 1;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output);
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set format on video encoder output port");
        if (encoder)
            mmal_component_destroy(encoder);
        state->encoder_component = NULL;
        return status;
    }

    if (state->encoding == MMAL_ENCODING_H264 &&
            state->intraperiod != -1)
    {
        MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set intraperiod");
            if (encoder)
                mmal_component_destroy(encoder);
            state->encoder_component = NULL;
            return status;
        }
    }

    if (state->encoding == MMAL_ENCODING_H264 &&
            state->quantisationParameter)
    {
        MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set initial QP");
            if (encoder)
                mmal_component_destroy(encoder);
            state->encoder_component = NULL;
            return status;
        }

        MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param2.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set min QP");
            if (encoder)
                mmal_component_destroy(encoder);
            state->encoder_component = NULL;
            return status;
        }

        MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param3.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set max QP");
            if (encoder)
                mmal_component_destroy(encoder);
            state->encoder_component = NULL;
            return status;
        }

    }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

      if((VCOS_ALIGN_UP(state->width,16) >> 4) * (VCOS_ALIGN_UP(state->height,16) >> 4) * state->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(state->width,16) >> 4) * (VCOS_ALIGN_UP(state->height,16) >> 4) * state->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            state->level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
		if (encoder)
		   mmal_component_destroy(encoder);
		state->encoder_component = NULL;
		return status;
         }
      }
      
      param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
		if (encoder)
		   mmal_component_destroy(encoder);
		state->encoder_component = NULL;
		return status;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   if (state->encoding == MMAL_ENCODING_H264)
   {
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE HEADER FLAG parameters");
         // Continue rather than abort..
      }

      //set flag for add SPS TIMING
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
         // Continue rather than abort..
      }

      //set INLINE VECTORS flag to request motion vector estimates
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE VECTORS parameters");
         // Continue rather than abort..
      }

      // Adaptive intra refresh settings
      if ( state->intra_refresh_type != -1)
      {
         MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
         param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
         param.hdr.size = sizeof(param);

         // Get first so we don't overwrite anything unexpectedly
         status = mmal_port_parameter_get(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
         }

         param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)state->intra_refresh_type;

         //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
         //   param.cir_mbs = 10;

         status = mmal_port_parameter_set(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to set H264 intra-refresh values");
		if (encoder)
		   mmal_component_destroy(encoder);
		state->encoder_component = NULL;
		return status;
         }
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
	if (encoder)
	   mmal_component_destroy(encoder);
	state->encoder_component = NULL;
	return status;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->encoder_pool = pool;
   state->encoder_component = encoder;

   if (state->verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;

}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
   if (signal_number == SIGUSR1)
   {
      // Handle but ignore - prevents us dropping out if started in none-signal mode
      // and someone sends us the USR1 signal anyway
   }
   else
   {
      // Going to abort on all other signals
      vcos_log_error("Aborting program\n");
      exit(130);
   }

}


/**
 * main
 */
int mipi327_cam_stream_run(CamSettings camset, void (*yuvCallback)(unsigned int w, unsigned int h, unsigned char * frame),
	       		   void (*nalCallback)(unsigned int len, unsigned char* nal)	){

   RASPIVID_STATE state;
   int exit_code = EX_OK;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_preview_port = NULL;
   MMAL_PORT_T *camera_video_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;
   MMAL_PORT_T *splitter_input_port = NULL;
   MMAL_PORT_T *splitter_encode_port = NULL;

   bcm_host_init();

   // Register our application with the logging system
   vcos_log_register("VeyeRaspiVid", VCOS_LOG_CATEGORY);

   signal(SIGINT, signal_handler);

   // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
   signal(SIGUSR1, SIG_IGN);

   default_status(camset, &state);

   // Do we have any parameters

   fprintf(stderr, "MIPI setting up for camera num %d\n",state.cameraNum);
   dump_status(&state);

   state.veye_camera_isp_state.sensor_mode = state.sensor_mode;
   if ((status = create_veye_camera_isp_component(&state.veye_camera_isp_state,state.cameraNum)) != MMAL_SUCCESS)
   {
       vcos_log_error("%s: Failed to create camera component", __func__);
       exit_code = EX_SOFTWARE;
   }
   else if ((status = create_encoder_component(&state)) != MMAL_SUCCESS)
   {
       vcos_log_error("%s: Failed to create encode component", __func__);
       destroy_veye_camera_isp_component(&state.veye_camera_isp_state);
       exit_code = EX_SOFTWARE;
   }
   else if ((status = create_splitter_component(&state, yuvCallback))!= MMAL_SUCCESS)
   {
       vcos_log_error("%s: Failed to create splitter component", __func__);
       destroy_encoder_component(&state);
       destroy_veye_camera_isp_component(&state.veye_camera_isp_state);

       exit_code = EX_SOFTWARE;
   }
   else
   {
       if (state.verbose)
           fprintf(stderr, "Starting component connection stage\n");

       camera_preview_port = state.veye_camera_isp_state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
       camera_video_port   = state.veye_camera_isp_state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
       printf("Got cam vid port?\n");
       encoder_input_port  = state.encoder_component->input[0];
       encoder_output_port = state.encoder_component->output[0];

       splitter_input_port = state.splitter_component->input[0];
       splitter_encode_port = state.splitter_component->output[SPLITTER_ENCODER_PORT];

       // Connect camera to preview
       if (state.verbose)
           fprintf(stderr, "Connecting camera video port to encoder input port\n");

       // Although we are not "previewing" - we need to connect the isp_component->input[0] to this port, for some reason...
       status = connect_ports(camera_preview_port, state.veye_camera_isp_state.isp_component->input[0], &state.isp_connection);
       if (status != MMAL_SUCCESS)
       {
           vcos_log_error("Failed to create rawcam->isp connection");
           goto error;
       } 	

       // Now connect the camera to the splitter
       status = connect_ports(state.veye_camera_isp_state.isp_component->output[0], splitter_input_port, &state.splitter_connection);
       if (status != MMAL_SUCCESS)
       {
           state.splitter_connection = NULL;
           vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
           goto error;
       }

       // Connect splitter to encoder
       status = connect_ports(splitter_encode_port, encoder_input_port, &state.encoder_connection);
       if (status != MMAL_SUCCESS)
       {
           state.encoder_connection = NULL;
           vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
           goto error;
       }


       // Set up our userdata - this is passed though to the callback where we need the information.
       state.nalState.nalCallback = nalCallback;
       state.nalState.pstate = &state;

       // Set up our userdata - this is passed though to the callback where we need the information.
       encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.nalState;

       if (state.verbose)
           fprintf(stderr, "Enabling encoder output port\n");

       // Enable the encoder output port and tell it its callback function
       status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

       if (status != MMAL_SUCCESS)
       {
           vcos_log_error("Failed to setup encoder output");
           goto error;
       }


       vcos_log_error("running now!!");
       int num = mmal_queue_length(state.encoder_pool->queue);
       int q;
       for (q=0;q<num;q++)
       {
           MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

           if (!buffer)
               vcos_log_error("Unable to get a required buffer %d from pool queue", q);

           if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
               vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
       }

       int captureEnable = 1;
       if (mmal_port_parameter_set_boolean(splitter_encode_port, MMAL_PARAMETER_CAPTURE, captureEnable) != MMAL_SUCCESS)
       {
           // How to handle?
       }

       while (1)
           // Have a sleep so we don't hog the CPU.
           vcos_sleep(10000);

error:

      mmal_status_to_int(status);

       fprintf(stderr, "mipi failure - Closing down\n");


      if (state.encoder_connection)
         mmal_connection_destroy(state.encoder_connection);
      if (state.splitter_connection)
         mmal_connection_destroy(state.splitter_connection);
      // Disable all our ports that are not handled by connections
   //   check_disable_port(camera_still_port);
      check_disable_port(encoder_output_port);
      // Can now close our file. Note disabling ports may flush buffers which causes
      // problems if we have already closed the file!
      if (state.encoder_component)
         mmal_component_disable(state.encoder_component);

      if (state.splitter_component)
         mmal_component_disable(state.splitter_component);
    	  if (state.veye_camera_isp_state.isp_component)
         mmal_component_disable(state.veye_camera_isp_state.isp_component);
      if (state.veye_camera_isp_state.camera_component)
         mmal_component_disable(state.veye_camera_isp_state.camera_component);
      destroy_encoder_component(&state);
      destroy_splitter_component(&state);
      destroy_veye_camera_isp_component(&state.veye_camera_isp_state);
      if (state.verbose)
         fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);
     
   }

   return exit_code;

}



