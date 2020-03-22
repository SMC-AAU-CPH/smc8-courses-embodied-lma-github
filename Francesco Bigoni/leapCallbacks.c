extern "C" {
#include "leapCallbacks.h"

	static LEAP_CONNECTION* connectionHandle;
	/** Callback for when the connection opens. */
	static void OnConnect()
	{
		printf("Connected.\n");
	}

	/** Callback for when a device is found. */
	static void OnDevice(const LEAP_DEVICE_INFO *props)
	{
		printf("Found device %s.\n", props->serial);
	}

	/** Callback for when a frame of tracking data is available. */
	static void OnFrame(const LEAP_TRACKING_EVENT *frame)
	{
		//printf("Frame %lli with %i hands.\n", (long long int)frame->info.frame_id, frame->nHands);

		for (uint32_t h = 0; h < frame->nHands; h++)
		{
			int j;
			LEAP_HAND* hand = &frame->pHands[h];

			if (hand->type == eLeapHandType_Left)
				j = 0;
			else
				j = 18;
			for (int f = 0; f < 5; f++)
			{
				LEAP_DIGIT finger = hand->digits[f];
				for (int c = 0; c < 3; c++)
					joints[j++] = finger.distal.next_joint.v[c];
			}
			
			for (int c = 0; c < 3; c++)
				joints[j++] = hand->palm.position.v[c];
				
		}

		frameReady = true;
		/*count++;
		if (count > 50)
		{
			for (int i = 0; i < 36; i++)
				printf("Joint %d: %f, ", i, joints[i]);
			printf("\n");
			count = 0;
		}*/
	}

	static void OnImage(const LEAP_IMAGE_EVENT *image)
	{
		const LEAP_IMAGE_PROPERTIES* properties = &image->image[0].properties;
		if (properties->bpp != 1)
			return;
		
		if (properties->width*properties->height != imageSize) 
		{
			void* prevImageBuffer = imageBuffer;
			imageWidth = properties->width;
			imageHeight = properties->height;
			imageSize = imageWidth * imageHeight;
			imageBytesPerPixel = properties->bpp;
			imageBuffer = malloc(2 * imageSize);
			if (prevImageBuffer)
				free(prevImageBuffer);
			textureChanged = true;
		}
		
		memcpy(imageBuffer, (char*)image->image[0].data + image->image[0].offset, imageSize);
		imageReady = true;

		//if (imageBuffer != nullptr && count > 100)
			//printf("Image buffer stored\n");
	}

	static void OnLogMessage(const eLeapLogSeverity severity, const int64_t timestamp, const char* message)
	{
		const char* severity_str;
		switch (severity)
		{
		case eLeapLogSeverity_Critical:
			severity_str = "Critical";
			break;
		case eLeapLogSeverity_Warning:
			severity_str = "Warning";
			break;
		case eLeapLogSeverity_Information:
			severity_str = "Info";
			break;
		default:
			severity_str = "";
			break;
		}
		printf("[%s][%lli] %s\n", severity_str, (long long int)timestamp, message);
	}

	static void* allocate(uint32_t size, eLeapAllocatorType typeHint, void* state)
	{
		void* ptr = malloc(size);
		return ptr;
	}

	static void deallocate(void* ptr, void* state)
	{
		if (!ptr)
			return;
		free(ptr);
	}

	void OnPointMappingChange(const LEAP_POINT_MAPPING_CHANGE_EVENT *change)
	{
		if (!connectionHandle)
			return;

		uint64_t size = 0;

		if (LeapGetPointMappingSize(*connectionHandle, &size) != eLeapRS_Success || !size)
			return;

		LEAP_POINT_MAPPING* pointMapping = (LEAP_POINT_MAPPING*)malloc(size);
		if (!pointMapping)
			return;

		if (LeapGetPointMapping(*connectionHandle, pointMapping, &size) == eLeapRS_Success && pointMapping->nPoints > 0)
		{
			printf("Managing %u points as of frame %lld at %lld\n", pointMapping->nPoints,
				(long long int)pointMapping->frame_id, (long long int)pointMapping->timestamp);
		}
		free(pointMapping);
	}

	void OnHeadPose(const LEAP_HEAD_POSE_EVENT *event)
	{

	}

	void initLeapMotion()
	{
		//Set callback function pointers
		ConnectionCallbacks.on_connection = &OnConnect;
		ConnectionCallbacks.on_device_found = &OnDevice;
		ConnectionCallbacks.on_frame = &OnFrame;
		ConnectionCallbacks.on_image = &OnImage;
		ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
		ConnectionCallbacks.on_log_message = &OnLogMessage;
		ConnectionCallbacks.on_head_pose = &OnHeadPose;

		connectionHandle = OpenConnection();
		{
			LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
			LeapSetAllocator(*connectionHandle, &allocator);
		}

		LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0);
	}
}