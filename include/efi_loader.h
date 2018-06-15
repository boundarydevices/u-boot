/*
 *  EFI application loader
 *
 *  Copyright (c) 2016 Alexander Graf
 *
 *  SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <part_efi.h>
#include <efi_api.h>

/* No need for efi loader support in SPL */
#if defined(CONFIG_EFI_LOADER) && !defined(CONFIG_SPL_BUILD)

#include <linux/list.h>

#define EFI_ENTRY(format, ...) do { \
	efi_restore_gd(); \
	debug("EFI: Entry %s(" format ")\n", __func__, ##__VA_ARGS__); \
	} while(0)

#define EFI_EXIT(ret) efi_exit_func(ret);

extern struct efi_runtime_services efi_runtime_services;
extern struct efi_system_table systab;

extern const struct efi_simple_text_output_protocol efi_con_out;
extern const struct efi_simple_input_interface efi_con_in;
extern const struct efi_console_control_protocol efi_console_control;

extern const efi_guid_t efi_guid_console_control;
extern const efi_guid_t efi_guid_device_path;
extern const efi_guid_t efi_guid_loaded_image;

extern unsigned int __efi_runtime_start, __efi_runtime_stop;
extern unsigned int __efi_runtime_rel_start, __efi_runtime_rel_stop;

/*
 * While UEFI objects can have callbacks, you can also call functions on
 * protocols (classes) themselves. This struct maps a protocol GUID to its
 * interface (usually a struct with callback functions).
 */
struct efi_class_map {
	const efi_guid_t *guid;
	const void *interface;
};

/*
 * When the UEFI payload wants to open a protocol on an object to get its
 * interface (usually a struct with callback functions), this struct maps the
 * protocol GUID to the respective protocol handler open function for that
 * object protocol combination.
 */
struct efi_handler {
	const efi_guid_t *guid;
	efi_status_t (EFIAPI *open)(void *handle,
			efi_guid_t *protocol, void **protocol_interface,
			void *agent_handle, void *controller_handle,
			uint32_t attributes);
};

/*
 * UEFI has a poor man's OO model where one "object" can be polymorphic and have
 * multiple different protocols (classes) attached to it.
 *
 * This struct is the parent struct for all of our actual implementation objects
 * that can include it to make themselves an EFI object
 */
struct efi_object {
	/* Every UEFI object is part of a global object list */
	struct list_head link;
	/* We support up to 4 "protocols" an object can be accessed through */
	struct efi_handler protocols[4];
	/* The object spawner can either use this for data or as identifier */
	void *handle;
};

/* This list contains all UEFI objects we know of */
extern struct list_head efi_obj_list;

/* Called by bootefi to make all disk storage accessible as EFI objects */
int efi_disk_register(void);
/* Called by bootefi to make GOP (graphical) interface available */
int efi_gop_register(void);
/* Called by bootefi to make the network interface available */
int efi_net_register(void **handle);
/* Called by bootefi to make SMBIOS tables available */
void efi_smbios_register(void);

/* Called by networking code to memorize the dhcp ack package */
void efi_net_set_dhcp_ack(void *pkt, int len);

/*
 * Stub implementation for a protocol opener that just returns the handle as
 * interface
 */
efi_status_t EFIAPI efi_return_handle(void *handle,
		efi_guid_t *protocol, void **protocol_interface,
		void *agent_handle, void *controller_handle,
		uint32_t attributes);
/* Called from places to check whether a timer expired */
void efi_timer_check(void);
/* PE loader implementation */
void *efi_load_pe(void *efi, struct efi_loaded_image *loaded_image_info);
/* Called once to store the pristine gd pointer */
void efi_save_gd(void);
/* Called from EFI_ENTRY on callback entry to put gd into the gd register */
void efi_restore_gd(void);
/* Called from EFI_EXIT on callback exit to restore the gd register */
efi_status_t efi_exit_func(efi_status_t ret);
/* Call this to relocate the runtime section to an address space */
void efi_runtime_relocate(ulong offset, struct efi_mem_desc *map);
/* Call this to set the current device name */
void efi_set_bootdev(const char *dev, const char *devnr, const char *path);

/* Generic EFI memory allocator, call this to get memory */
void *efi_alloc(uint64_t len, int memory_type);
/* More specific EFI memory allocator, called by EFI payloads */
efi_status_t efi_allocate_pages(int type, int memory_type, unsigned long pages,
				uint64_t *memory);
/* EFI memory free function. */
efi_status_t efi_free_pages(uint64_t memory, unsigned long pages);
/* EFI memory allocator for small allocations */
efi_status_t efi_allocate_pool(int pool_type, unsigned long size,
			       void **buffer);
/* EFI pool memory free function. */
efi_status_t efi_free_pool(void *buffer);
/* Returns the EFI memory map */
efi_status_t efi_get_memory_map(unsigned long *memory_map_size,
				struct efi_mem_desc *memory_map,
				unsigned long *map_key,
				unsigned long *descriptor_size,
				uint32_t *descriptor_version);
/* Adds a range into the EFI memory map */
uint64_t efi_add_memory_map(uint64_t start, uint64_t pages, int memory_type,
			    bool overlap_only_ram);
/* Called by board init to initialize the EFI memory map */
int efi_memory_init(void);
/* Adds new or overrides configuration table entry to the system table */
efi_status_t efi_install_configuration_table(const efi_guid_t *guid, void *table);

#ifdef CONFIG_EFI_LOADER_BOUNCE_BUFFER
extern void *efi_bounce_buffer;
#define EFI_LOADER_BOUNCE_BUFFER_SIZE (64 * 1024 * 1024)
#endif

/* Convert strings from normal C strings to uEFI strings */
static inline void ascii2unicode(u16 *unicode, const char *ascii)
{
	while (*ascii)
		*(unicode++) = *(ascii++);
}

/*
 * Use these to indicate that your code / data should go into the EFI runtime
 * section and thus still be available when the OS is running
 */
#define __efi_runtime_data __attribute__ ((section ("efi_runtime_data")))
#define __efi_runtime __attribute__ ((section ("efi_runtime_text")))

/* Call this with mmio_ptr as the _pointer_ to a pointer to an MMIO region
 * to make it available at runtime */
void efi_add_runtime_mmio(void *mmio_ptr, u64 len);

/* Boards may provide the functions below to implement RTS functionality */

void __efi_runtime EFIAPI efi_reset_system(
			enum efi_reset_type reset_type,
			efi_status_t reset_status,
			unsigned long data_size, void *reset_data);
void efi_reset_system_init(void);

efi_status_t __efi_runtime EFIAPI efi_get_time(
			struct efi_time *time,
			struct efi_time_cap *capabilities);
void efi_get_time_init(void);

#else /* defined(EFI_LOADER) && !defined(CONFIG_SPL_BUILD) */

/* Without CONFIG_EFI_LOADER we don't have a runtime section, stub it out */
#define __efi_runtime_data
#define __efi_runtime
static inline void efi_add_runtime_mmio(void *mmio_ptr, u64 len) { }

/* No loader configured, stub out EFI_ENTRY */
static inline void efi_restore_gd(void) { }
static inline void efi_set_bootdev(const char *dev, const char *devnr,
				   const char *path) { }
static inline void efi_net_set_dhcp_ack(void *pkt, int len) { }

#endif
