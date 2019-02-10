//////////////////////////////////////////////////////////////////////////////
// COPYRIGHT (c) 2009 Schweitzer Engineering Laboratories, Inc.
///
/// @file sel_class.c
///
/// @brief devfs support for sel devices.
//////////////////////////////////////////////////////////////////////////////

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>

#include <linux/version.h>

/// Class structure to hold our own class information
static struct class *sel_class;

//////////////////////////////////////////////////////////////////////////////
/// @brief Register a device with devfs
///
/// @param devt Device id for the device
/// @param name string containing device name
//////////////////////////////////////////////////////////////////////////////
void sel_register( dev_t devt, char const *name, ... )
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
   char s[512];
#endif
   va_list vargs;
   va_start(vargs, name);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
   vsnprintf( s, sizeof(s), name, vargs );
   device_create( sel_class, NULL, devt, s );
#else
   device_create_vargs( sel_class, NULL, devt, NULL, name, vargs );
#endif
   va_end(vargs);
}
EXPORT_SYMBOL(sel_register);

//////////////////////////////////////////////////////////////////////////////
/// @brief Unregister a device with devfs
///
/// @param devt Device id for the device
//////////////////////////////////////////////////////////////////////////////
void sel_deregister( dev_t devt )
{
   device_destroy( sel_class, devt );
}
EXPORT_SYMBOL(sel_deregister);

//////////////////////////////////////////////////////////////////////////////
/// @brief Initializer to create the device class
///
/// @return 0 for success, PTR_ERR of the class pointer on failure
//////////////////////////////////////////////////////////////////////////////
static int __init sel_init(void)
{
   int rc = 0;

   // create the device class
   sel_class = class_create(THIS_MODULE, "sel");
   if( IS_ERR(sel_class) )
   {
      rc = PTR_ERR(sel_class);
   }

   return rc;
}
subsys_initcall(sel_init);

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Schweitzer Engineering Laboratories, Inc." );
MODULE_DESCRIPTION( "sysfs class for SEL devices." );
MODULE_VERSION( "1.0.0" );

// vim:sm:et:sw=3:sts=3
