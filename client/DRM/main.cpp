/**
 * FreeRDP: A Remote Desktop Protocol Implementation
 * FreeRDP DRM Client
 *
 * Copyright 2026 Victor Ding <victor@ding.pm>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <freerdp/config.h>

#include <freerdp/freerdp.h>
#include <freerdp/gdi/gdi.h>
#include <freerdp/client/cmdline.h>
#include <freerdp/constants.h>
#include <freerdp/log.h>
#include <winpr/assert.h>

#include <drm/drm.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/input.h>
#include <poll.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <libudev.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <system_error>
#include <vector>

#define TAG CLIENT_TAG("DRM")

namespace checked
{
	inline int open(const char* path, int oflag)
	{
		auto ret = ::open(path, oflag);
		if (ret < 0)
		{
			perror("Error: ");
			throw std::system_error(errno, std::generic_category());
		}
		return ret;
	}
	inline void* mmap(void* addr, size_t len, int prot, int flags, int fd, __off_t offset)
	{
		auto ret = ::mmap(addr, len, prot, flags, fd, offset);
		if (ret == MAP_FAILED)
		{
			perror("Error: ");
			throw std::system_error(errno, std::generic_category());
		}
		return ret;
	}
	inline void mprotect(void* __addr, size_t __len, int __prot)
	{
		if (::mprotect(__addr, __len, __prot) != 0)
		{
			perror("Error: ");
			throw std::system_error(errno, std::generic_category());
		}
	}
	inline void ioctl(int fd, unsigned long int request, void* args)
	{
		if (::ioctl(fd, request, args) != 0)
		{
			perror("Error: ");
			throw std::system_error(errno, std::generic_category());
		}
	}
	inline void freerdp_settings_set_uint32(rdpSettings* settings, FreeRDP_Settings_Keys_UInt32 id,
	                                        UINT32 val)
	{
		if (!::freerdp_settings_set_uint32(settings, id, val))
			throw std::exception();
	}
	inline void freerdp_settings_set_bool(rdpSettings* settings, FreeRDP_Settings_Keys_Bool id,
	                                      BOOL val)
	{
		if (!::freerdp_settings_set_bool(settings, id, val))
			throw std::exception();
	}
}

#pragma pack(push, 1)
class FrameBuffer
{
	// Arg 0. nop               ; keep rdi, untouched
	// Arg 1. lea rsi, [rip - 7]; address of the FrameBuffer
	uint8_t instr0[7] = { 0x48, 0x8D, 0x35, 0xF9, 0xFF, 0xFF, 0xFF };
	//        jmp [rip]         ; jump to the real function
	uint8_t instr1[6] = { 0xFF, 0x25, 0x00, 0x00, 0x00, 0x00 };
	void* jmp_target = nullptr;

  public:
	void* addr = nullptr;
	size_t size = 0;
	uint32_t pitch = 0;
	uint32_t handle = 0;
	uint32_t fb_id = 0;
	int drm_fd = 0;

	using TDeleter = void (*)(void*);
	inline TDeleter deleter() const
	{
		return (TDeleter)this;
	}
	inline static FrameBuffer* New(int drm_fd, uint32_t width, uint32_t height)
	{
		auto buffer = static_cast<FrameBuffer*>(checked::mmap(nullptr, sizeof(FrameBuffer),
		                                                      PROT_READ | PROT_WRITE,
		                                                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
		new (buffer) FrameBuffer();
		buffer->jmp_target = (void*)Delete;
		buffer->drm_fd = drm_fd;

		struct drm_mode_create_dumb create = { 0 };
		create.width = width;
		create.height = height;
		create.bpp = 32;
		checked::ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create);
		buffer->pitch = create.pitch;
		buffer->handle = create.handle;

		struct drm_mode_fb_cmd fb = { 0 };
		fb.width = create.width;
		fb.height = create.height;
		fb.pitch = create.pitch;
		fb.bpp = create.bpp;
		fb.depth = 24;
		fb.handle = create.handle;
		checked::ioctl(drm_fd, DRM_IOCTL_MODE_ADDFB, &fb);

		buffer->fb_id = fb.fb_id;

		struct drm_mode_map_dumb map = { 0 };
		map.handle = create.handle;
		checked::ioctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map);
		buffer->size = create.size;

		buffer->addr =
		    checked::mmap(0, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
		checked::mprotect(buffer, sizeof(FrameBuffer), PROT_READ | PROT_EXEC);
		return buffer;
	}
	static void Delete(void* addr, FrameBuffer* self)
	{
		if (!self || self->addr != addr)
			return;
		if (addr)
		{
			::munmap(addr, self->size);
		}
		if (self->fb_id)
		{
			::ioctl(self->drm_fd, DRM_IOCTL_MODE_RMFB, &self->fb_id);
		}
		if (self->handle)
		{
			struct drm_mode_destroy_dumb destroy = { .handle = self->handle };
			::ioctl(self->drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
		}
		::munmap(self, sizeof(*self));
	}
};
#pragma pack(pop)

class LinuxFile
{
  protected:
	int fd;

  public:
	inline LinuxFile(const char* path, int oflag)
	{
		fd = checked::open(path, oflag);
	}
	inline ~LinuxFile()
	{
		close(fd);
	}
	template <unsigned long int request> inline void ioctl(void* args) const
	{
		checked::ioctl(fd, request, args);
	}
	template <bool allow_partial = false> inline ssize_t write(const void* buf, size_t n) const
	{
		auto nw = ::write(fd, buf, n);
		if (!allow_partial && nw != n)
			throw std::system_error(errno, std::generic_category());
		return nw;
	}
	template <bool allow_partial = false> inline ssize_t read(void* buf, size_t n) const
	{
		auto nr = ::read(fd, buf, n);
		if (!allow_partial && nr != n)
			throw std::system_error(errno, std::generic_category());
		return nr;
	}
	inline int raw()
	{
		return fd;
	}
};

struct DumbBuffer : public drm_mode_create_dumb
{
	void* data;
	inline int create(int drm_fd, uint32_t width, uint32_t height)
	{
		memset(this, 0, sizeof(*this));
		this->width = width;
		this->height = height;
		bpp = 32;
		int ret = 0;

		ret = ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, this);
		if (ret < 0)
		{
			perror("DRM_IOCTL_MODE_CREATE_DUMB");
			return ret;
		}
		struct drm_mode_map_dumb map = { .handle = handle };
		ret = ioctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map);
		if (ret < 0)
		{
			perror("DRM_IOCTL_MODE_MAP_DUMB");
		}
		data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
		if (!data)
			ret = -1;
		return ret;
	}
	inline void destroy(int drm_fd)
	{
		if (data)
		{
			munmap(data, size);
		}
		if (handle)
		{
			struct drm_mode_destroy_dumb destroy = { .handle = handle };
			ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
		}
	}
};

class DRM : public LinuxFile
{
  private:
	uint32_t crtc_id;

  public: // TODO: hack
	DumbBuffer cursor;

  private:
	class Connector
	{
	  public:
		uint32_t connector_id;
		std::optional<struct drm_mode_modeinfo> mode;
		std::vector<uint32_t> encoders;
		inline Connector() = default;
		inline Connector(const DRM& drm, uint32_t connector_id) : connector_id(connector_id), mode()
		{
			struct drm_mode_get_connector conn = { 0 };
			conn.connector_id = connector_id;
			drm.ioctl<DRM_IOCTL_MODE_GETCONNECTOR>(&conn);
			// DRM_MODE_CONNECTED         = 1,
			// DRM_MODE_DISCONNECTED      = 2,
			// DRM_MODE_UNKNOWNCONNECTION = 3
			if (conn.connection != 1)
				return;
			if (conn.count_modes == 0)
				return;
			if (conn.count_encoders == 0)
				return;

			std::vector<struct drm_mode_modeinfo> modes(conn.count_modes);
			conn.modes_ptr = (__u64)(void*)modes.data();
			encoders.resize(conn.count_encoders);
			conn.encoders_ptr = (__u64)(void*)encoders.data();
			conn.count_props = 0;
			if (conn.count_modes == 0)
				return;
			drm.ioctl<DRM_IOCTL_MODE_GETCONNECTOR>(&conn);
			if (!modes.empty())
				mode = modes[0];
		}
	};

  public:
	// TODO: enumerate all /dev/dri/card[n]?
	inline DRM(size_t monitor_id)
	    : LinuxFile(
#ifdef __amd64__
	          "/dev/dri/card0"
#else
	          "/dev/dri/card1"
#endif
	          ,
	          O_RDWR)
	{
		struct drm_mode_card_res res = { 0 };
		ioctl<DRM_IOCTL_MODE_GETRESOURCES>(&res);

		if (res.count_connectors == 0)
			return;
		std::vector<uint32_t> connector_ids(res.count_connectors);
		res.connector_id_ptr = (__u64)(void*)connector_ids.data();
		res.count_crtcs = 0;
		res.count_encoders = 0;
		res.count_fbs = 0;
		ioctl<DRM_IOCTL_MODE_GETRESOURCES>(&res);

		std::vector<Connector> monitors;
		for (const auto& connector_id : connector_ids)
		{
			Connector conn(*this, connector_id);
			if (conn.mode.has_value())
				monitors.push_back(conn);
		}
		if (monitors.size() == 0)
		{
			printf("No  monitors found.\n");
			throw std::exception();
		}
		if (monitors.size() <= monitor_id)
		{
			printf("Available monitors:\n");
			for (size_t i = 0; i < monitors.size(); i++)
			{
				const auto& mode = monitors[i].mode;
				printf("\t[%zu]: %ux%u\n", i, mode->hdisplay, mode->vdisplay);
			}
			throw std::exception();
		}
		monitor = std::move(monitors[monitor_id]);
	}
	inline FrameBuffer* initialize()
	{
		crtc_id = 0;
		for (auto encoder : monitor.encoders)
		{
			struct drm_mode_get_encoder enc = { 0 };
			enc.encoder_id = encoder;
			ioctl<DRM_IOCTL_MODE_GETENCODER>(&enc);
			if (enc.crtc_id != 0)
			{
				crtc_id = enc.crtc_id;
				break;
			}
		}
		if (!crtc_id)
		{
			fprintf(stderr, "Could not find CRTC\n");
			throw std::exception();
		}

		auto& mode = monitor.mode.value();
		auto frame_buffer = FrameBuffer::New(raw(), mode.hdisplay, mode.vdisplay);

		struct drm_mode_crtc crtc = { 0 };
		crtc.crtc_id = crtc_id;
		crtc.fb_id = frame_buffer->fb_id;
		crtc.set_connectors_ptr = (__u64)(void*)&monitor.connector_id;
		crtc.count_connectors = 1;
		crtc.mode_valid = 1;
		crtc.mode = mode;
		ioctl<DRM_IOCTL_MODE_SETCRTC>(&crtc);

		// Query maximum supported cursor width and height
		cursor.create(raw(), GetCap(DRM_CAP_CURSOR_WIDTH), GetCap(DRM_CAP_CURSOR_HEIGHT));
		memset(cursor.data, 0, cursor.size);

		struct drm_mode_cursor arg = { 0 };
		arg.flags = DRM_MODE_CURSOR_BO;
		arg.crtc_id = crtc_id;
		arg.width = cursor.width;
		arg.height = cursor.height;
		arg.handle = cursor.handle;
		ioctl<DRM_IOCTL_MODE_CURSOR>(&arg);

		return frame_buffer;
	}
	inline void deinitialize()
	{
		struct drm_mode_cursor arg = { 0 };
		arg.flags = DRM_MODE_CURSOR_BO;
		arg.crtc_id = crtc_id;
		ioctl<DRM_IOCTL_MODE_CURSOR>(&arg);
		cursor.destroy(raw());
	}
	inline void RefreshCursor()
	{
		struct drm_mode_cursor cursor = { 0 };
		cursor.flags = DRM_MODE_CURSOR_MOVE;
		cursor.crtc_id = crtc_id;
		cursor.x = std::clamp(x - hot_x, 0, monitor.mode->hdisplay - 1);
		cursor.y = std::clamp(y - hot_y, 0, monitor.mode->vdisplay - 1);
		ioctl<DRM_IOCTL_MODE_CURSOR>(&cursor);
	}

	Connector monitor;
	// Cursor
	int hot_x = 0;
	int hot_y = 0;
	int x = 0;
	int y = 0;

  private:
	inline uint64_t GetCap(uint64_t capability)
	{
		struct drm_get_cap cap = { 0 };
		cap.capability = capability;
		ioctl<DRM_IOCTL_GET_CAP>(&cap);
		return cap.value;
	}
};

class drmContext : rdpClientContext
{
  public:
	DRM drm;
	inline void OnInput(struct input_event event)
	{
		switch (event.type)
		{
			case EV_SYN:
				// Ignore
				break;
			case EV_KEY:
				OnKEY(event);
				break;
			case EV_REL:
				OnREL(event);
				break;
			case EV_MSC:
				// Ignore
				break;
			default:
				printf("Unknown input type %d, code %d, value %d\n", event.type, event.code,
				       event.value);
		}
	}
	inline void OnKEY(struct input_event event)
	{
		switch (event.code)
		{
			case BTN_LEFT:
			{
				UINT16 flags = (event.value == 0 ? 0 : PTR_FLAGS_DOWN) | PTR_FLAGS_BUTTON1;
				freerdp_input_send_mouse_event(context.input, flags, drm.x, drm.y);
				break;
			}
			case BTN_RIGHT:
			{
				UINT16 flags = (event.value == 0 ? 0 : PTR_FLAGS_DOWN) | PTR_FLAGS_BUTTON2;
				freerdp_input_send_mouse_event(context.input, flags, drm.x, drm.y);
				break;
			}
			case BTN_MIDDLE:
			{
				UINT16 flags = (event.value == 0 ? 0 : PTR_FLAGS_DOWN) | PTR_FLAGS_BUTTON3;
				freerdp_input_send_mouse_event(context.input, flags, drm.x, drm.y);
				break;
			}
			default:
			{
				auto scancode = GetVirtualScanCodeFromVirtualKeyCode(
				    GetVirtualKeyCodeFromKeycode(event.code, WINPR_KEYCODE_TYPE_EVDEV),
				    WINPR_KBD_TYPE_IBM_ENHANCED);
				freerdp_input_send_keyboard_event_ex(context.input, event.value != 0,
				                                     event.value == 2, scancode);
			}
		}
	}
	inline void OnREL(struct input_event event)
	{
		switch (event.code)
		{
			case REL_X:
			{
				drm.x = std::clamp(drm.x + 2 * event.value, 0, (int)drm.monitor.mode->hdisplay);
				freerdp_input_send_mouse_event(context.input, PTR_FLAGS_MOVE, drm.x, drm.y);
				drm.RefreshCursor();
				break;
			}
			case REL_Y:
			{
				drm.y = std::clamp(drm.y + 2 * event.value, 0, (int)drm.monitor.mode->vdisplay);
				freerdp_input_send_mouse_event(context.input, PTR_FLAGS_MOVE, drm.x, drm.y);
				drm.RefreshCursor();
				break;
			}
			case REL_HWHEEL:
			case REL_WHEEL:
				// Ignore, use REL_WHEEL_HI_RES & REL_HWHEEL_HI_RES
				break;
			case REL_WHEEL_HI_RES:
			{
				UINT16 value = WheelRotationMask & (UINT16)event.value;
				freerdp_input_send_mouse_event(context.input, value | PTR_FLAGS_WHEEL, 0, 0);
				break;
			}
			case REL_HWHEEL_HI_RES:
			{
				UINT16 value = WheelRotationMask & (UINT16)event.value;
				freerdp_input_send_mouse_event(context.input, value | PTR_FLAGS_HWHEEL, 0, 0);
				break;
			}
			default:
				printf("Unknown REL: code %d, value %d\n", event.code, event.value);
		}
	}
};

template <typename F0, typename F1> class ScopeGuard
{
  public:
	explicit ScopeGuard(F0 constructor, F1 destructor) : destructor(std::move(destructor))
	{
		constructor();
	}
	~ScopeGuard()
	{
		destructor();
	}
	ScopeGuard(const ScopeGuard&) = delete;
	ScopeGuard& operator=(const ScopeGuard&) = delete;

  private:
	F1 destructor;
};

class EvDev : protected LinuxFile
{
  public:
	inline EvDev(const char* path) : LinuxFile(path, O_RDWR | O_NONBLOCK)
	{
		ioctl<EVIOCGRAB>((void*)1u);
	}
	virtual std::optional<struct input_event> event()
	{
		struct input_event event = { 0 };
		if (read<true>(&event, sizeof(event)) == sizeof(event))
			return event;
		else
			return {};
	}
};

class InputManager
{
  public:
	InputManager()
	    : udev(udev_new(), &udev_unref),
	      mon(udev_monitor_new_from_netlink(udev.get(), "udev"), &udev_monitor_unref)
	{
		std::unique_ptr<struct udev_enumerate, decltype(&udev_enumerate_unref)> enumerate(
		    udev_enumerate_new(udev.get()), &udev_enumerate_unref);
		udev_enumerate_add_match_subsystem(enumerate.get(), "input");
		udev_enumerate_add_match_property(enumerate.get(), "ID_BUS", "usb");
		udev_enumerate_scan_devices(enumerate.get());

		// Scan for existing keyboards
		struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate.get());
		struct udev_list_entry* entry;
		udev_list_entry_foreach(entry, devices)
		{
			const char* path = udev_list_entry_get_name(entry);
			std::unique_ptr<struct udev_device, decltype(&udev_device_unref)> dev(
			    udev_device_new_from_syspath(udev.get(), path), &udev_device_unref);
			process_device(dev.get());
		}

		udev_monitor_filter_add_match_subsystem_devtype(mon.get(), "input", NULL);
		udev_monitor_enable_receiving(mon.get());
	}
	template <typename F> inline void check_fds(F on_event)
	{
		for (auto& dev : devices)
		{
			auto event = dev.second->event();
			if (event.has_value())
				on_event(event.value());
		}
		// Hot-plug
		std::unique_ptr<struct udev_device, decltype(&udev_device_unref)> dev(
		    udev_monitor_receive_device(mon.get()), &udev_device_unref);
		process_device(dev.get());
	}

  private:
	std::map<std::string, std::unique_ptr<EvDev>> devices;
	std::unique_ptr<struct udev, decltype(&udev_unref)> udev;
	std::unique_ptr<struct udev_monitor, decltype(&udev_monitor_unref)> mon;
	void process_device(struct udev_device* dev)
	{
		if (!dev)
			return;

		const char* devnode = udev_device_get_devnode(dev);
		const char* action = udev_device_get_action(dev);
		const char* is_kbd = udev_device_get_property_value(dev, "ID_INPUT_KEYBOARD");
		const char* is_mouse = udev_device_get_property_value(dev, "ID_INPUT_MOUSE");

		if (!devnode || !strstr(devnode, "event"))
			return;

		if ((is_kbd && strcmp(is_kbd, "1") == 0) || (is_mouse && strcmp(is_mouse, "1") == 0))
		{
			if (action && strstr(action, "remove"))
			{
				devices.erase(devnode);
			}
			else
			{
				devices[devnode] = std::make_unique<EvDev>(devnode);
			}
			printf("%s: %s\n", action ? action : "EXISTING", devnode);
		}
	}
};

int main(int argc, char* argv[])
{
	RDP_CLIENT_ENTRY_POINTS EntryPoints = { 0 };
	EntryPoints.Version = RDP_CLIENT_INTERFACE_VERSION;
	EntryPoints.Size = sizeof(RDP_CLIENT_ENTRY_POINTS);
	EntryPoints.ContextSize = sizeof(drmContext);
	EntryPoints.ClientNew = [](freerdp* instance, rdpContext* context) -> BOOL
	{
		instance->PreConnect = [](freerdp* instance) -> BOOL
		{
			auto settings = instance->context->settings;
			auto monitor_id = 0u;
			if (freerdp_settings_get_bool(settings, FreeRDP_ListMonitors))
				monitor_id = std::numeric_limits<decltype(monitor_id)>::max();
			else if (freerdp_settings_get_uint32(settings, FreeRDP_NumMonitorIds) > 0)
				monitor_id =
				    *(uint32_t*)freerdp_settings_get_pointer_array(settings, FreeRDP_MonitorIds, 0);

			auto context = reinterpret_cast<drmContext*>(instance->context);
			new (&context->drm) DRM(monitor_id);

			checked::freerdp_settings_set_uint32(settings, FreeRDP_DesktopWidth,
			                                     context->drm.monitor.mode->hdisplay);
			checked::freerdp_settings_set_uint32(settings, FreeRDP_DesktopHeight,
			                                     context->drm.monitor.mode->vdisplay);
			checked::freerdp_settings_set_bool(settings, FreeRDP_Fullscreen, TRUE);
			checked::freerdp_settings_set_bool(settings, FreeRDP_GfxThinClient, TRUE);
			checked::freerdp_settings_set_bool(settings, FreeRDP_MultiTouchInput, TRUE);
			checked::freerdp_settings_set_bool(settings, FreeRDP_CertificateCallbackPreferPEM,
			                                   TRUE);
			checked::freerdp_settings_set_uint32(settings, FreeRDP_OsMajorType, OSMAJORTYPE_UNIX);
			checked::freerdp_settings_set_uint32(settings, FreeRDP_OsMinorType,
			                                     OSMINORTYPE_UNSPECIFIED);
			PubSub_SubscribeChannelConnected(instance->context->pubSub,
			                                 freerdp_client_OnChannelConnectedEventHandler);
			PubSub_SubscribeChannelDisconnected(instance->context->pubSub,
			                                    freerdp_client_OnChannelDisconnectedEventHandler);
			return TRUE;
		};
		instance->PostConnect = [](freerdp* instance) -> BOOL
		{
			auto buffer = reinterpret_cast<drmContext*>(instance->context)->drm.initialize();
			if (!gdi_init_ex(instance, PIXEL_FORMAT_BGRA32, buffer->pitch, (BYTE*)buffer->addr,
			                 buffer->deleter()))
				return FALSE;

			rdpPointer pointer = { 0 };
			pointer.size = sizeof(rdpPointer) +
			               reinterpret_cast<drmContext*>(instance->context)->drm.cursor.size;
			pointer.New = [](rdpContext* context, rdpPointer* pointer) -> BOOL
			{
				// Data is appended to pointer
				return freerdp_image_copy_from_pointer_data(
				    (BYTE*)(void*)(pointer + 1), PIXEL_FORMAT_BGRA32,
				    reinterpret_cast<drmContext*>(context)->drm.cursor.pitch, 0, 0, pointer->width,
				    pointer->height, pointer->xorMaskData, pointer->lengthXorMask,
				    pointer->andMaskData, pointer->lengthAndMask, pointer->xorBpp,
				    &context->gdi->palette);
			};
			pointer.Set = [](rdpContext* context, rdpPointer* pointer) -> BOOL
			{
				reinterpret_cast<drmContext*>(context)->drm.hot_x = pointer->xPos;
				reinterpret_cast<drmContext*>(context)->drm.hot_y = pointer->yPos;
				const auto& cursor = reinterpret_cast<drmContext*>(context)->drm.cursor;
				// Data is appended to pointer
				memcpy(cursor.data, (void*)(pointer + 1), cursor.size);
				return TRUE;
			};
			pointer.SetNull = [](rdpContext* context) -> BOOL
			{
				const auto& cursor = reinterpret_cast<drmContext*>(context)->drm.cursor;
				memset(cursor.data, 0, cursor.size);
				return TRUE;
			};
			pointer.SetDefault = pointer.SetNull;
			graphics_register_pointer(instance->context->graphics, &pointer);

			instance->context->update->DesktopResize = [](rdpContext* context) -> BOOL
			{
				return reinterpret_cast<drmContext*>(context)->drm.monitor.mode->hdisplay ==
				           freerdp_settings_get_uint32(context->settings, FreeRDP_DesktopWidth) &&
				       reinterpret_cast<drmContext*>(context)->drm.monitor.mode->vdisplay ==
				           freerdp_settings_get_uint32(context->settings, FreeRDP_DesktopHeight);
			};
			return TRUE;
		};
		instance->PostDisconnect = [](freerdp* instance)
		{
			reinterpret_cast<drmContext*>(instance->context)->drm.deinitialize();
			gdi_free(instance);
			reinterpret_cast<drmContext*>(instance->context)->drm.~DRM();
		};
		return TRUE;
	};

	std::unique_ptr<rdpContext, decltype(&freerdp_client_context_free)> context(
	    freerdp_client_context_new(&EntryPoints), freerdp_client_context_free);
	// command line
	{
		auto status =
		    freerdp_client_settings_parse_command_line(context->settings, argc, argv, FALSE);
		if (status)
		{
			(void)freerdp_client_settings_command_line_status_print(context->settings, status, argc,
			                                                        argv);
		}
		(void)stream_dump_register_handlers(context.get(), CONNECTION_STATE_MCS_CREATE_REQUEST,
		                                    FALSE);
	}

	ScopeGuard Client([&]() { (void)freerdp_client_start(context.get()); },
	                  [&]() { (void)freerdp_client_stop(context.get()); });
	ScopeGuard Instance([&]() { (void)freerdp_connect(context->instance); },
	                    [&]() { (void)freerdp_disconnect(context->instance); });

	InputManager inputs;
	while (!freerdp_shall_disconnect_context(context.get()))
	{
		inputs.check_fds([&](struct input_event event)
		                 { reinterpret_cast<drmContext*>(context.get())->OnInput(event); });
		if (!freerdp_check_event_handles(context.get()))
			break;
		sched_yield();
	}

	return 0;
}
